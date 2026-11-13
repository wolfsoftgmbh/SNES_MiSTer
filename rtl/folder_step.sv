//
// folder_step -- example user of the Main_MiSTer "folder as playlist" feature.
//
// Copyright (c) 2026 Klaus Wolf
//
// This source file is free software: you can redistribute it and/or modify
// it under the terms of the GNU General Public License as published
// by the Free Software Foundation, either version 3 of the License, or
// (at your option) any later version.
//
// This source file is distributed in the hope that it will be useful,
// but WITHOUT ANY WARRANTY; without even the implied warranty of
// MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.  See the
// GNU General Public License for more details.
//
// You should have received a copy of the GNU General Public License
// along with this program.  If not, see <http://www.gnu.org/licenses/>.
//
///////////////////////////////////////////////////////////////////////
//
// The core asks Main for the next / previous / a random file of the folder the
// current file came from. Two UIO commands travel over EXT_BUS of hps_io:
//
//   0x47  file request (core -> Main). Main polls it. Answer to the command word:
//         {8'd0 (slot), 4'hA, counter[3:0]}. Main only looks at a changed counter.
//         Then one data word follows: [3:0] request code
//         (0 none, 1 next, 2 previous, 3 random, 4 first - 4 is not used here).
//   0x48  folder state (Main -> core). Three data words (position, count, flags).
//         This example ignores them; the module does not answer this command.
//
// Counter: 4 bit, advances once per new request that was not yet fetched. Several
// requests before a poll give one request with the last code. Reading the data word
// marks the request as fetched; a further poll returns the same counter and code 0.
//
// Buttons (joy = joy0 of SNES.sv, bits from the CONF_STR line
// "J1,A,B,X,Y,LT,RT,Select,Start,SaveState": A=4 B=5 X=6 Y=7 L=8 R=9 Select=10 Start=11):
//   Select + R -> next, Select + L -> previous, Select + X -> random.
// One request per press (rising edge of the combination). Nothing is requested while
// the OSD is open or when the option is off; holding the buttons through an OSD
// close does not fire either.
//
// ---------------------------------------------------------------------------
// Sharing EXT_BUS with hps_ext (MSU-1) -- wiring pattern for the top level
// ---------------------------------------------------------------------------
// hps_io has ONE EXT_BUS port. hps_io drives [31:16] (word from Main), [33] (strobe),
// [34] (io_enable), [35]; the core drives [15:0] (answer) and [32] ("I answer").
// hps_ext assigns [15:0] and [32] unconditionally, so two modules cannot both be
// connected to the same wire (multiple drivers). Give each module its own bus,
// feed the hps_io side to both, and merge only the answer side:
//
//   wire [35:0] EXT_BUS;                       // to hps_io
//   wire [35:0] EXT_BUS_MSU, EXT_BUS_FOLDER;   // one private bus per module
//
//   assign EXT_BUS_MSU   [35:33] = EXT_BUS[35:33];   // hps_io side as input to both;
//   assign EXT_BUS_MSU   [31:16] = EXT_BUS[31:16];   // bit 32 and [15:0] are the
//   assign EXT_BUS_FOLDER[35:33] = EXT_BUS[35:33];   // module's own outputs
//   assign EXT_BUS_FOLDER[31:16] = EXT_BUS[31:16];
//
//   assign EXT_BUS[32]   = EXT_BUS_MSU[32] | EXT_BUS_FOLDER[32];
//   assign EXT_BUS[15:0] = EXT_BUS_FOLDER[32] ? EXT_BUS_FOLDER[15:0] : EXT_BUS_MSU[15:0];
//
// The command ranges are disjoint (hps_ext: 0x34..0x35, folder_step:
// 0x47), so at most one module raises its bit 32; the select bit makes that explicit.
// The merged bit 32 is what hps_io uses to take [15:0] from the core (hps_io.sv:194).
//
// All of this is checked by werkzeuge/snes_folder_test (shared-bus test).
//
module folder_step (
	input             clk,
	input             reset,      // active high
	inout      [35:0] EXT_BUS,

	input             enable,     // OSD option
	input             osd_open,
	input      [12:0] joy         // joy0 of SNES.sv
);

wire        io_strobe = EXT_BUS[33];
wire        io_enable = EXT_BUS[34];
wire [15:0] io_din    = EXT_BUS[31:16];

localparam [15:0] CMD_REQUEST = 16'h0047;

localparam [3:0] CODE_NEXT   = 4'd1;
localparam [3:0] CODE_PREV   = 4'd2;
localparam [3:0] CODE_RANDOM = 4'd3;

// ---------------------------------------------------------------- buttons

wire combo_next   = joy[10] & joy[9];  // Select + R
wire combo_prev   = joy[10] & joy[8];  // Select + L
wire combo_random = joy[10] & joy[6];  // Select + X

reg  [2:0] combo_old;
wire [2:0] combo_rise = {combo_random, combo_prev, combo_next} & ~combo_old;

wire       req_valid = enable & ~osd_open & |combo_rise;
wire [3:0] req_code  = combo_rise[0] ? CODE_NEXT :
                       combo_rise[1] ? CODE_PREV : CODE_RANDOM;

// ---------------------------------------------------------------- bus side

reg  [7:0]  byte_cnt;
reg  [15:0] cmd;
reg  [15:0] io_dout;
reg         dout_en;

assign EXT_BUS[15:0] = io_dout;
assign EXT_BUS[32]   = dout_en;

reg  [3:0]  counter;
reg         pending;     // request waiting, not fetched yet
reg  [3:0]  code;

// the data word of 0x47 fetches the request (in the clock of its strobe)
wire fetch        = io_enable & io_strobe & (cmd == CMD_REQUEST) & (byte_cnt == 8'd1);
wire pending_rest = pending & ~fetch;

always @(posedge clk) begin
	combo_old <= {combo_random, combo_prev, combo_next};

	if (reset) begin
		byte_cnt <= 8'd0;
		cmd      <= 16'd0;
		dout_en  <= 1'b0;
		io_dout  <= 16'd0;
		counter  <= 4'd0;
		pending  <= 1'b0;
		code     <= 4'd0;
	end else begin
		if (!io_enable) begin
			cmd      <= 16'd0;
			dout_en  <= 1'b0;
			byte_cnt <= 8'd0;
			io_dout  <= 16'd0;
		end else if (io_strobe) begin
			io_dout <= 16'd0;
			if (byte_cnt != 8'hFF) byte_cnt <= byte_cnt + 8'd1;

			if (byte_cnt == 8'd0) begin
				cmd <= io_din;
				if (io_din == CMD_REQUEST) begin
					dout_en <= 1'b1;
					io_dout <= {8'd0, 4'hA, counter};
				end
			end else if (cmd == CMD_REQUEST && byte_cnt == 8'd1) begin
				io_dout <= {12'd0, pending ? code : 4'd0};
			end
		end

		// A new request while one is still pending only replaces the code; otherwise
		// it is a new counter step. One that coincides with the fetch counts as new.
		if (req_valid) begin
			code <= req_code;
			if (!pending_rest) counter <= counter + 4'd1;
			pending <= 1'b1;
		end else begin
			pending <= pending_rest;
		end
	end
end

// EXT_BUS[35] and [15:0] (own output) are not read here
wire _unused_ok = &{1'b0, EXT_BUS[35], joy[12:11], joy[7], joy[5:0], 1'b0};

endmodule
