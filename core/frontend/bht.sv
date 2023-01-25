// Copyright 2018 - 2019 ETH Zurich and University of Bologna.
// Copyright and related rights are licensed under the Solderpad Hardware
// License, Version 2.0 (the "License"); you may not use this file except in
// compliance with the License.  You may obtain a copy of the License at
// http://solderpad.org/licenses/SHL-2.0. Unless required by applicable law
// or agreed to in writing, software, hardware and materials distributed under
// this License is distributed on an "AS IS" BASIS, WITHOUT WARRANTIES OR
// CONDITIONS OF ANY KIND, either express or implied. See the License for the
// specific language governing permissions and limitations under the License.
//
// Author: Florian Zaruba, ETH Zurich
// Date: 08.02.2018
// Migrated: Luis Vitorio Cargnini, IEEE
// Date: 09.06.2018

// branch history table - 2 bit saturation counter
module bht #(
    parameter int unsigned NR_ENTRIES = 1024
)(
    input  logic                        clk_i,
    input  logic                        rst_ni,
    input  logic                        flush_i,
    input  logic                        debug_mode_i,
    input  logic [riscv::VLEN-1:0]      vpc_i,
    input  ariane_pkg::bht_update_t     bht_update_i,
    input  logic                        enable_i,
    input  logic [63:0]                 checkpoint_addr_i,
    // we potentially need INSTR_PER_FETCH predictions/cycle
    output ariane_pkg::bht_prediction_t [ariane_pkg::INSTR_PER_FETCH-1:0] bht_prediction_o,
    output ariane_pkg::dcache_req_i_t   bht_checkpoint_o, //output to Dcache
    input  ariane_pkg::dcache_req_o_t   bht_checkpoint_i,
    output logic                        rst_checkpoint_o //output to CSR to reset 0x808 to 0 after all data has been checkpointed
);
    // the last bit is always zero, we don't need it for indexing
    localparam OFFSET = ariane_pkg::RVC == 1'b1 ? 1 : 2;
    // re-shape the branch history table
    localparam NR_ROWS = NR_ENTRIES / ariane_pkg::INSTR_PER_FETCH;
    // number of bits needed to index the row
    localparam ROW_ADDR_BITS = $clog2(ariane_pkg::INSTR_PER_FETCH);
    localparam ROW_INDEX_BITS = ariane_pkg::RVC == 1'b1 ? $clog2(ariane_pkg::INSTR_PER_FETCH) : 1;   // 1    1
    // number of bits we should use for prediction
    localparam PREDICTION_BITS = $clog2(NR_ROWS) + OFFSET + ROW_ADDR_BITS;
    // we are not interested in all bits of the address
    unread i_unread (.d_i(|vpc_i));

    localparam ENTRIES_PER_FETCH = riscv::XLEN / 4; //4 bits per entry, XLEN bits per cache fetch
    localparam ROWS_PER_FETCH = ENTRIES_PER_FETCH / ariane_pkg::INSTR_PER_FETCH; //how many rows can we fill at each time

    struct packed {
        logic       valid;
        logic [1:0] saturation_counter;
    } bht_d[NR_ROWS-1:0][ariane_pkg::INSTR_PER_FETCH-1:0], bht_q[NR_ROWS-1:0][ariane_pkg::INSTR_PER_FETCH-1:0];
    
    logic [$clog2(NR_ENTRIES):0] checkpoint_counter_d, checkpoint_counter_q;
    ariane_pkg::dcache_req_i_t     checkpoint_request_d, checkpoint_request_q;
    logic                          csr_reset_d, csr_reset_q;

    logic [7:0] rows_completed_q, rows_completed_d;

    logic [$clog2(NR_ROWS)-1:0]  index, update_pc;
    logic [ROW_INDEX_BITS-1:0]    update_row_index;
    logic [1:0]                  saturation_counter;

    assign index     = vpc_i[PREDICTION_BITS - 1:ROW_ADDR_BITS + OFFSET];
    assign update_pc = bht_update_i.pc[PREDICTION_BITS - 1:ROW_ADDR_BITS + OFFSET];
    if (ariane_pkg::RVC) begin : gen_update_row_index
      assign update_row_index = bht_update_i.pc[ROW_ADDR_BITS + OFFSET - 1:OFFSET];
    end else begin
      assign update_row_index = '0;
    end

    // prediction assignment
    for (genvar i = 0; i < ariane_pkg::INSTR_PER_FETCH; i++) begin : gen_bht_output
        assign bht_prediction_o[i].valid = (enable_i) ? bht_q[index][i].valid : 1'b0;
        assign bht_prediction_o[i].taken = (enable_i) ? bht_q[index][i].saturation_counter[1] == 1'b1 : 1'b0;
    end

    //assignments for checkpointing
    assign bht_checkpoint_o = checkpoint_request_q;
    assign rst_checkpoint_o = csr_reset_q;

    always_comb begin : update_bht
        if(enable_i) begin
            bht_d = bht_q;
            saturation_counter = bht_q[update_pc][update_row_index].saturation_counter;

            if (bht_update_i.valid && !debug_mode_i) begin
                bht_d[update_pc][update_row_index].valid = 1'b1;

                if (saturation_counter == 2'b11) begin
                    // we can safely decrease it
                    if (!bht_update_i.taken)
                        bht_d[update_pc][update_row_index].saturation_counter = saturation_counter - 1;
                // then check if it saturated in the negative regime e.g.: branch not taken
                end else if (saturation_counter == 2'b00) begin
                    // we can safely increase it
                    if (bht_update_i.taken)
                        bht_d[update_pc][update_row_index].saturation_counter = saturation_counter + 1;
                end else begin // otherwise we are not in any boundaries and can decrease or increase it
                    if (bht_update_i.taken)
                        bht_d[update_pc][update_row_index].saturation_counter = saturation_counter + 1;
                    else
                        bht_d[update_pc][update_row_index].saturation_counter = saturation_counter - 1;
                end
            end
//don't directly assign into bht_d below, assign it into a logic and then write it through here
        end else begin //turned off, microwaving
            //assign to bht_d
            bht_d = bht_q;
            rows_completed_d = rows_completed_q;
            checkpoint_counter_d = checkpoint_counter_q;

            if (bht_checkpoint_i.data_rvalid) begin //new batch of data
                checkpoint_counter_d = checkpoint_counter_q + ENTRIES_PER_FETCH;
                rows_completed_d = rows_completed_q + ROWS_PER_FETCH; 
                
                for(int i = 0; i < ROWS_PER_FETCH; i++) begin
                    for(int j = 0; j < ariane_pkg::INSTR_PER_FETCH; j++) begin
                        bht_d[rows_completed_q+i][j] = bht_checkpoint_i.data_rdata[riscv::XLEN - (4*ariane_pkg::INSTR_PER_FETCH*i + (4*ariane_pkg::INSTR_PER_FETCH - 4)*j) - 4  +: 3];
                    end
                end
            end
        end
    end

    typedef enum logic[1:0] {IDLE, SEND_INDEX, SEND_TAG} state_e;
    state_e ckpt_state_d, ckpt_state_q;

    logic tag_written_d, tag_written_q;

    logic [ariane_pkg::DCACHE_TAG_WIDTH]   ckpt_tag_d, ckpt_tag_q;
    logic [ariane_pkg::DCACHE_INDEX_WIDTH] ckpt_index_d, ckpt_index_q;

    always_comb begin : checkpointing_bht_fsm//not enabled, make this into a state machine 
        //step 1 IDLE
        //step 2 SEND INDEX
        //step 3 SEND TAG
        ckpt_state_d         = ckpt_state_q;
        checkpoint_request_d = checkpoint_request_q;
        ckpt_tag_d           = ckpt_tag_q;
        ckpt_index_d         = ckpt_index_q;
        csr_reset_d          = csr_reset_q;
        tag_written_d        = tag_written_q;
        case(ckpt_state_q)
            IDLE: begin
                csr_reset_d = 0; 
                if(!rst_checkpoint_o) begin
                    ckpt_state_d = SEND_INDEX;
                end else begin
                    ckpt_state_d = IDLE;
                end
            end
            SEND_INDEX: begin 
                //change req
                checkpoint_request_d.tag_valid = 1'b0;
                ckpt_state_d = SEND_INDEX;

                //checkpoint_request_d.address_tag = checkpoint_addr_i[63:63-ariane_pkg::DCACHE_TAG_WIDTH];
                //checkpoint_request_d.address_index = checkpoint_addr_i[63-ariane_pkg::DCACHE_TAG_WIDTH-1:63-ariane_pkg::DCACHE_TAG_WIDTH-1-ariane_pkg::DCACHE_INDEX_WIDTH] + 4*checkpoint_counter_q;

                checkpoint_request_d.address_tag = ckpt_tag_q;
                checkpoint_request_d.address_index = ckpt_index_q;
                checkpoint_request_d.data_size = 2'b10;
                checkpoint_request_d.data_be = '1;

                if(checkpoint_counter_q == NR_ENTRIES) begin
                    ckpt_state_d = IDLE;
                    csr_reset_d = 1;
                end else begin
                    //send req
                    checkpoint_request_d.data_req = 1'b1;
                    tag_written_d = 0;
                    if(bht_checkpoint_i.data_gnt)begin
                        ckpt_state_d = SEND_TAG;
                    end
                end
            end
            SEND_TAG: begin 
                //send tag_valid with tag
                checkpoint_request_d.data_req = 1'b0;
                tag_written_d = 1;
                if(tag_written_q == 0) begin
                    checkpoint_request_d.tag_valid = 1'b1;
                end else begin
                    checkpoint_request_d.tag_valid = 1'b0;
                end
                if(bht_checkpoint_i.data_rvalid) begin
                    ckpt_state_d = SEND_INDEX;
                    ckpt_index_d = ckpt_index_q + 8;
                    tag_written_d = 0;
                end else begin
                    ckpt_state_d = SEND_TAG;
                end
            end
        endcase
    end

    always_ff @(posedge clk_i or negedge rst_ni) begin
        if (!rst_ni) begin
            for (int unsigned i = 0; i < NR_ROWS; i++) begin
                for (int j = 0; j < ariane_pkg::INSTR_PER_FETCH; j++) begin
                    bht_q[i][j] <= '0;
                end
            end
            checkpoint_counter_q <= 0;
            checkpoint_request_q <= '0;
            ckpt_state_q <= IDLE;
            rows_completed_q <= 0;
            ckpt_tag_q <= '0;
            ckpt_index_q <= '0;
            tag_written_q <= 0;
        end else if(enable_i) begin
            // evict all entries
            if (flush_i) begin
                for (int i = 0; i < NR_ROWS; i++) begin
                    for (int j = 0; j < ariane_pkg::INSTR_PER_FETCH; j++) begin
                        bht_q[i][j].valid <=  1'b0;
                        bht_q[i][j].saturation_counter <= 2'b10;
                    end
                end
            end else begin
                bht_q <= bht_d;
            end
            checkpoint_counter_q <= 0;
            checkpoint_request_q <= '0;
            csr_reset_q <= 0;
            tag_written_q <= 0;
            rows_completed_q <= 0;
            ckpt_tag_q <= checkpoint_addr_i[ariane_pkg::DCACHE_INDEX_WIDTH+ariane_pkg::DCACHE_TAG_WIDTH-1:ariane_pkg::DCACHE_INDEX_WIDTH];
            ckpt_index_q <= checkpoint_addr_i[ariane_pkg::DCACHE_INDEX_WIDTH-1:0];
        end else begin //not reset and not enabled
            checkpoint_request_q <= checkpoint_request_d;
            checkpoint_counter_q <= checkpoint_counter_d;
            csr_reset_q <= csr_reset_d;
            ckpt_state_q <= ckpt_state_d;
            rows_completed_q <= rows_completed_d;
            ckpt_tag_q <= ckpt_tag_d;
            ckpt_index_q <= ckpt_index_d;
            tag_written_q <= tag_written_d;
            bht_q <= bht_d;
        end
    end
endmodule
