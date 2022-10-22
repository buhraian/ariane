/* Copyright 2018 ETH Zurich and University of Bologna.
 * Copyright and related rights are licensed under the Solderpad Hardware
 * License, Version 0.51 (the "License"); you may not use this file except in
 * compliance with the License.  You may obtain a copy of the License at
 * http://solderpad.org/licenses/SHL-0.51. Unless required by applicable law
 * or agreed to in writing, software, hardware and materials distributed under
 * this License is distributed on an "AS IS" BASIS, WITHOUT WARRANTIES OR
 * CONDITIONS OF ANY KIND, either express or implied. See the License for the
 * specific language governing permissions and limitations under the License.
 *
 * File: $filename.v
 *
 * Description: Auto-generated bootrom
 */

// Auto-generated code
module bootrom (
   input  logic         clk_i,
   input  logic         req_i,
   input  logic [63:0]  addr_i,
   output logic [63:0]  rdata_o
);
    localparam int RomSize = 215;

    const logic [RomSize-1:0][63:0] mem = {
        64'h00000000_00766564,
        64'h6e2c7663_73697200,
        64'h79746972_6f697270,
        64'h2d78616d_2c766373,
        64'h69720073_656d616e,
        64'h2d676572_00646564,
        64'h6e657478_652d7374,
        64'h70757272_65746e69,
        64'h00746669_68732d67,
        64'h65720073_74707572,
        64'h7265746e_6900746e,
        64'h65726170_2d747075,
        64'h72726574_6e690064,
        64'h65657073_2d746e65,
        64'h72727563_00736567,
        64'h6e617200_656c646e,
        64'h61687000_72656c6c,
        64'h6f72746e_6f632d74,
        64'h70757272_65746e69,
        64'h00736c6c_65632d74,
        64'h70757272_65746e69,
        64'h23007469_6c70732d,
        64'h626c7400_65707974,
        64'h2d756d6d_00617369,
        64'h2c766373_69720073,
        64'h75746174_73006765,
        64'h72006570_79745f65,
        64'h63697665_64007963,
        64'h6e657571_6572662d,
        64'h6b636f6c_63007963,
        64'h6e657571_6572662d,
        64'h65736162_656d6974,
        64'h006c6564_6f6d0065,
        64'h6c626974_61706d6f,
        64'h6300736c_6c65632d,
        64'h657a6973_2300736c,
        64'h6c65632d_73736572,
        64'h64646123_09000000,
        64'h02000000_02000000,
        64'h02000000_01000000,
        64'ha9000000_04000000,
        64'h03000000_01000000,
        64'h1d010000_04000000,
        64'h03000000_07000000,
        64'h0a010000_04000000,
        64'h03000000_00000004,
        64'h00000000_000010f1,
        64'hff000000_5b000000,
        64'h10000000_03000000,
        64'h09000000_02000000,
        64'h0b000000_02000000,
        64'hec000000_10000000,
        64'h03000000_94000000,
        64'h00000000_03000000,
        64'h00306369_6c702c76,
        64'h63736972_1b000000,
        64'h0c000000_03000000,
        64'h01000000_83000000,
        64'h04000000_03000000,
        64'h00000000_00000000,
        64'h04000000_03000000,
        64'h00303030_30303131,
        64'h66666640_63696c70,
        64'h01000000_02000000,
        64'h006c6f72_746e6f63,
        64'h00010000_08000000,
        64'h03000000_00000c00,
        64'h00000000_000002f1,
        64'hff000000_5b000000,
        64'h10000000_03000000,
        64'h07000000_02000000,
        64'h03000000_02000000,
        64'hec000000_10000000,
        64'h03000000_00000000,
        64'h30746e69_6c632c76,
        64'h63736972_1b000000,
        64'h0d000000_03000000,
        64'h00000000_30303030,
        64'h32303166_66664074,
        64'h6e696c63_01000000,
        64'h02000000_006c6f72,
        64'h746e6f63_00010000,
        64'h08000000_03000000,
        64'h00100000_00000000,
        64'h000000f1_ff000000,
        64'h5b000000_10000000,
        64'h03000000_ffff0000,
        64'h02000000_ec000000,
        64'h08000000_03000000,
        64'h00333130_2d677562,
        64'h65642c76_63736972,
        64'h1b000000_10000000,
        64'h03000000_00303030,
        64'h30303031_66666640,
        64'h72656c6c_6f72746e,
        64'h6f632d67_75626564,
        64'h01000000_02000000,
        64'h00000000_e2000000,
        64'h04000000_03000000,
        64'h01000000_d7000000,
        64'h04000000_03000000,
        64'h01000000_c6000000,
        64'h04000000_03000000,
        64'h00c20100_b8000000,
        64'h04000000_03000000,
        64'h80f0fa02_3f000000,
        64'h04000000_03000000,
        64'h00400d00_00000000,
        64'h00c0c2f0_ff000000,
        64'h5b000000_10000000,
        64'h03000000_00303535,
        64'h3631736e_1b000000,
        64'h08000000_03000000,
        64'h00303030_63326330,
        64'h66666640_74726175,
        64'h01000000_b1000000,
        64'h00000000_03000000,
        64'h00007375_622d656c,
        64'h706d6973_00636f73,
        64'h2d657261_622d656e,
        64'h61697261_2c687465,
        64'h1b000000_1f000000,
        64'h03000000_02000000,
        64'h0f000000_04000000,
        64'h03000000_02000000,
        64'h00000000_04000000,
        64'h03000000_00636f73,
        64'h01000000_02000000,
        64'h00000040_00000000,
        64'h00000080_00000000,
        64'h5b000000_10000000,
        64'h03000000_00007972,
        64'h6f6d656d_4f000000,
        64'h07000000_03000000,
        64'h00303030_30303030,
        64'h38407972_6f6d656d,
        64'h01000000_02000000,
        64'h02000000_02000000,
        64'h02000000_a9000000,
        64'h04000000_03000000,
        64'h00006374_6e692d75,
        64'h70632c76_63736972,
        64'h1b000000_0f000000,
        64'h03000000_94000000,
        64'h00000000_03000000,
        64'h01000000_83000000,
        64'h04000000_03000000,
        64'h00000000_72656c6c,
        64'h6f72746e_6f632d74,
        64'h70757272_65746e69,
        64'h01000000_79000000,
        64'h00000000_03000000,
        64'h00003933_76732c76,
        64'h63736972_70000000,
        64'h0b000000_03000000,
        64'h00006364_66616d69,
        64'h34367672_66000000,
        64'h0b000000_03000000,
        64'h00000076_63736972,
        64'h00656e61_69726120,
        64'h2c687465_1b000000,
        64'h12000000_03000000,
        64'h00000000_79616b6f,
        64'h5f000000_05000000,
        64'h03000000_00000000,
        64'h5b000000_04000000,
        64'h03000000_00757063,
        64'h4f000000_04000000,
        64'h03000000_80f0fa02,
        64'h3f000000_04000000,
        64'h03000000_00000030,
        64'h40757063_01000000,
        64'he1f50500_2c000000,
        64'h04000000_03000000,
        64'h00000000_0f000000,
        64'h04000000_03000000,
        64'h01000000_00000000,
        64'h04000000_03000000,
        64'h00000000_73757063,
        64'h01000000_00657261,
        64'h622d656e_61697261,
        64'h2c687465_26000000,
        64'h10000000_03000000,
        64'h00766564_2d657261,
        64'h622d656e_61697261,
        64'h2c687465_1b000000,
        64'h14000000_03000000,
        64'h02000000_0f000000,
        64'h04000000_03000000,
        64'h02000000_00000000,
        64'h04000000_03000000,
        64'h00000000_01000000,
        64'h00000000_00000000,
        64'h00000000_00000000,
        64'hd4040000_28010000,
        64'h00000000_10000000,
        64'h11000000_28000000,
        64'h0c050000_38000000,
        64'h34060000_edfe0dd0,
        64'h00000000_00000000,
        64'h00000000_00000000,
        64'h00000000_00000000,
        64'h00000000_00000000,
        64'h00000000_00000000,
        64'h00000000_0000bff5,
        64'h10500073_03c58593,
        64'h00000597_f1402573,
        64'h00000000_00000000,
        64'h00000000_00000000,
        64'h00000000_00000000,
        64'h00000000_00000000,
        64'h00000000_00000000,
        64'h00000000_84020765,
        64'h85930000_0597f140,
        64'h2573047e_0010041b
    };

    logic [$clog2(RomSize)-1:0] addr_q;

    always_ff @(posedge clk_i) begin
        if (req_i) begin
            addr_q <= addr_i[$clog2(RomSize)-1+3:3];
        end
    end

    // this prevents spurious Xes from propagating into
    // the speculative fetch stage of the core
    assign rdata_o = (addr_q < RomSize) ? mem[addr_q] : '0;
endmodule
