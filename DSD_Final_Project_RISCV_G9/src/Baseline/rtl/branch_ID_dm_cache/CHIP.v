// Top module of your design, you cannot modify this module!!
// `include "./RISCV_hasHazard.v"
// `include "./FORWARDING_UNIT.v"
// `include "./HAZARD_DETECTION_UNIT.v"
// `include "./cache_dm.v"
// `include "./cache_ro.v"
module CHIP (	clk,
				rst_n,
//----------for slow_memD------------
				mem_read_D,
				mem_write_D,
				mem_addr_D,
				mem_wdata_D,
				mem_rdata_D,
				mem_ready_D,
//----------for slow_memI------------
				mem_read_I,
				mem_write_I,
				mem_addr_I,
				mem_wdata_I,
				mem_rdata_I,
				mem_ready_I,
//----------for TestBed--------------				
				DCACHE_addr, 
				DCACHE_wdata,
				DCACHE_wen   
			);
input			clk, rst_n;
//--------------------------

output			mem_read_D;
output			mem_write_D;
output	[31:4]	mem_addr_D;
output	[127:0]	mem_wdata_D;
input	[127:0]	mem_rdata_D;
input			mem_ready_D;
//--------------------------
output			mem_read_I;
output			mem_write_I;
output	[31:4]	mem_addr_I;
output	[127:0]	mem_wdata_I;
input	[127:0]	mem_rdata_I;
input			mem_ready_I;
//----------for TestBed--------------
output	[29:0]	DCACHE_addr;
output	[31:0]	DCACHE_wdata;
output			DCACHE_wen;
//--------------------------

// wire declaration
wire        ICACHE_ren;
wire        ICACHE_wen;
wire [29:0] ICACHE_addr;
wire [31:0] ICACHE_wdata;
wire        ICACHE_stall;
wire [31:0] ICACHE_rdata;

wire        DCACHE_ren;
wire        DCACHE_wen;
wire [29:0] DCACHE_addr;
wire [31:0] DCACHE_wdata;
wire        DCACHE_stall;
wire [31:0] DCACHE_rdata;


//=========================================
	// Note that the overall design of your RISCV includes:
	// 1. pipelined RISCV processor
	// 2. data cache
	// 3. instruction cache


	RISCV_Pipeline i_RISCV(
		// control interface
		.clk            (clk)           , 
		.rst_n          (rst_n)         ,
//----------I cache interface-------		
		.ICACHE_ren     (ICACHE_ren)    ,
		.ICACHE_addr    (ICACHE_addr)   ,
        .ICACHE_wen     (ICACHE_wen)    ,
		.ICACHE_stall   (ICACHE_stall)  ,
		.ICACHE_rdata   (ICACHE_rdata)  ,
//----------D cache interface-------
		.DCACHE_ren     (DCACHE_ren)    ,
		.DCACHE_wen     (DCACHE_wen)    ,
		.DCACHE_addr    (DCACHE_addr)   ,
		.DCACHE_wdata   (DCACHE_wdata)  ,
		.DCACHE_stall   (DCACHE_stall)  ,
		.DCACHE_rdata   (DCACHE_rdata)
	);
	

	cache D_cache(
        .clk        (clk)         ,
        .proc_reset (~rst_n)      ,
        .proc_read  (DCACHE_ren)  ,
        .proc_write (DCACHE_wen)  ,
        .proc_addr  (DCACHE_addr) ,
        .proc_rdata (DCACHE_rdata),
        .proc_wdata (DCACHE_wdata),
        .proc_stall (DCACHE_stall),
        .mem_read   (mem_read_D)  ,
        .mem_write  (mem_write_D) ,
        .mem_addr   (mem_addr_D)  ,
        .mem_wdata  (mem_wdata_D) ,
        .mem_rdata  (mem_rdata_D) ,
        .mem_ready  (mem_ready_D)
	);

	cache_read_only I_cache(
        .clk        (clk)         ,
        .proc_reset (~rst_n)      ,
        .proc_read  (ICACHE_ren)  ,
        .proc_write (ICACHE_wen)  ,
        .proc_addr  (ICACHE_addr) ,
        .proc_rdata (ICACHE_rdata),
        .proc_stall (ICACHE_stall),
        .mem_read   (mem_read_I)  ,
		.mem_write	(mem_write_I) ,
        .mem_addr   (mem_addr_I)  ,
		.mem_wdata	(mem_wdata_I) ,
        .mem_rdata  (mem_rdata_I) ,
        .mem_ready  (mem_ready_I)
	);
endmodule

// Five Stage pilpeline RISCV CPU

module RISCV_Pipeline(
    // control interface
    clk,
    rst_n,
//----------I cache interface-------
    ICACHE_ren      ,
    ICACHE_wen      ,
	ICACHE_addr     ,
	ICACHE_stall    ,
	ICACHE_rdata    ,
//----------D cache interface-------
	DCACHE_ren      ,
	DCACHE_wen      ,
	DCACHE_addr     ,
	DCACHE_wdata    ,
	DCACHE_stall    ,
	DCACHE_rdata   
);

//==== I/O Definition ====================
    // control interface ---------------
    input   clk;
    input   rst_n;
    //----------I cache interface-------
    output          ICACHE_ren;
    output          ICACHE_wen;
	output  [29:0]  ICACHE_addr;
	input           ICACHE_stall;
	input   [31:0]  ICACHE_rdata;
    //----------D cache interface-------
    output          DCACHE_ren;
    output	        DCACHE_wen;
	output  [29:0]  DCACHE_addr;
	output  [31:0]  DCACHE_wdata;
	input           DCACHE_stall;
	input   [31:0]  DCACHE_rdata;     

//==== Pipeline Register Definition =====
    //---- IF/ID ------------------------
    wire [31:0]  IFID_pc_addr_w;
    wire [31:0]  IFID_inst_w; 
    reg  [31:0]  IFID_pc_addr_r;
    reg  [31:0]  IFID_inst_r; 
    //---- ID/EX ------------------------
    //WB regwrite, memToreg
    wire regwrite;
    wire memtoreg;
    wire IDEX_RegWrite_w;
    wire IDEX_MemToReg_w;
    reg  IDEX_RegWrite_r;
    reg  IDEX_MemToReg_r;
    //M  jal, jalr, branch, memread, memwrite
    wire jal;
    wire jalr;
    wire branch;
    wire memread;
    wire memwrite;

    //wire IDEX_jal_w;
    //wire IDEX_jalr_w; 
    //wire IDEX_branch_w;
    wire IDEX_MemRead_w;
    wire IDEX_MemWrite_w;
    wire IDEX_jump_w;
    //reg  IDEX_jal_r;
    //reg  IDEX_jalr_r; 
    //reg  IDEX_branch_r;
    reg  IDEX_MemRead_r;
    reg  IDEX_MemWrite_r;
    reg  IDEX_jump_r;
    //EX aluop, alusrc
    wire [1:0]   aluop;
    wire         alusrc;
    wire [1:0]   IDEX_aluop_w;
    wire         IDEX_alusrc_w;
    reg  [4:0]   IDEX_rs1_r;
    reg  [4:0]   IDEX_rs2_r;
    wire [4:0]   IDEX_rs1_w;
    wire [4:0]   IDEX_rs2_w;
    wire [31:0]  IDEX_pc_addr_w;
    wire [31:0]  IDEX_rdata1_w;
    wire [31:0]  IDEX_rdata2_w;
    reg  [1:0]   IDEX_aluop_r;
    reg          IDEX_alusrc_r;
    reg  [31:0]  IDEX_pc_addr_r;
    reg  [31:0]  IDEX_rdata1_r;
    reg  [31:0]  IDEX_rdata2_r;
    // other data
    wire [31:0]  IDEX_imm_w;
    wire         IDEX_func7_w;
    wire [2:0]   IDEX_func3_w;
    wire [4:0]   IDEX_rd_addr_w;
    reg  [31:0]  IDEX_imm_r;
    reg          IDEX_func7_r;
    reg  [2:0]   IDEX_func3_r;
    reg  [4:0]   IDEX_rd_addr_r;  

    //---- EX/MEM -----------------------
    //WB regwrite, memToreg
    wire EXMEM_RegWrite_w;
    wire EXMEM_MemToReg_w;
    reg  EXMEM_RegWrite_r;
    reg  EXMEM_MemToReg_r;
    //M  jal, jalr, branch, memread, memwrite
    //wire EXMEM_jal_w;
    //wire EXMEM_jalr_w; 
    //wire EXMEM_branch_w;
    wire EXMEM_jump_w;
    wire EXMEM_MemRead_w;
    wire EXMEM_MemWrite_w;
    //reg  EXMEM_jal_r;
    //reg  EXMEM_jalr_r; 
    //reg  EXMEM_branch_r;
    reg  EXMEM_MemRead_r;
    reg  EXMEM_MemWrite_r;
    reg  EXMEM_jump_r;
    // pc addr
    //wire [29:0]  EXMEM_jalr_addr_w;
    //wire [29:0]  EXMEM_branch_or_jal_addr_w;
    // reg  [29:0]  EXMEM_jalr_addr_r;
    // reg  [29:0]  EXMEM_branch_or_jal_addr_r;
    // alu result
    wire [31:0]  EXMEM_alu_out_w;
    // wire         EXMEM_alu_zero_w;
    wire [31:0]  EXMEM_wdata_w;
    reg  [31:0]  EXMEM_alu_out_r;
    // reg          EXMEM_alu_zero_r;
    reg  [31:0]  EXMEM_wdata_r;
    // rd_addr
    wire [4:0]   EXMEM_rd_addr_w;
    reg  [4:0]   EXMEM_rd_addr_r;
    // for jal, jalr
    wire [31:0]  EXMEM_pc_add4_w;
    reg  [31:0]  EXMEM_pc_add4_r;
 
    wire [3:0]   alu_ctrl;                         

    //---- MEM/WB -----------------------
    //WB regwrite, memToreg
    wire MEMWB_RegWrite_w;
    wire MEMWB_MemToReg_w;
    reg  MEMWB_RegWrite_r;
    reg  MEMWB_MemToReg_r;
    // alu result
    wire [31:0] MEMWB_alu_out_w;
    wire [31:0] MEMWB_rdata_w;
    reg  [31:0] MEMWB_alu_out_r;
    reg  [31:0] MEMWB_rdata_r;
    // rd_addr
    wire [4:0]   MEMWB_rd_addr_w;
    reg  [4:0]   MEMWB_rd_addr_r;
    // for jal, jalr
    wire [31:0]  MEMWB_pc_add4_w;
    reg  [31:0]  MEMWB_pc_add4_r;
    //wire         MEMWB_jal_w;
    //reg          MEMWB_jal_r;
    //wire         MEMWB_jalr_w;
    //reg          MEMWB_jalr_r;

    //---- Hazard detection ----------
    wire         hazard_stall;
    wire [1:0]   forward_a_ex; //ex stage
    wire [1:0]   forward_b_ex;
    wire [1:0]   forward_a_id; //id stage
    wire [1:0]   forward_b_id;

    // choose data from forwarding unit 
    wire  [31:0]    forwarding_x;
    wire  [31:0]    forwarding_y; 

//==== BRANCH/JUMP in ID stage =============================
    assign ICACHE_ren = 1'b1;
    assign ICACHE_wen = 1'b0;
    // its new !!
    wire [31:0] busX;
    wire [31:0] wdata;
    wire [31:0] ID_jal_or_branch_addr;
    // wire [31:0] ID_jalr_addr[0:2];
    wire [31:0] ID_real_jalr_addr;

    wire ID_beq;
    wire ID_branch_jump;
    //wire ID_jump;

    // assign ID_real_jalr_addr = (forward_a_id[1]) ?  ID_jalr_addr[1] :
    //                            (forward_a_id[0]) ?  ID_jalr_addr[2] :
    //                                                 ID_jalr_addr[0] ; 
    wire [31:0] forward_jalr;

    assign ID_jal_or_branch_addr = IFID_pc_addr_r + IDEX_imm_w;
    assign ID_real_jalr_addr = forward_jalr + IDEX_imm_w;
    assign forward_jalr =   (forward_a_id[1]) ?  EXMEM_alu_out_r :
                            (forward_a_id[0]) ?  wdata :
                                                 busX ; 

    assign ID_beq = (IDEX_rdata1_w == IDEX_rdata2_w);
    assign ID_branch_jump = ((ID_beq ^ IFID_inst_r[12]) && IFID_inst_r[6] && (~IFID_inst_r[2]));
    assign IDEX_jump_w = (jal|jalr);

    // ID_jump id_jump(
    //     .pc                  ( IFID_pc_addr_r),
    //     .offset              ( IDEX_imm_w ),
    //     .rdata1              ( busX ),
    //     .forward_data1       ( EXMEM_alu_out_r ),
    //     .forward_data2       ( wdata),
    //     .jal_or_branch_addr  (ID_jal_or_branch_addr),
    //     .jalr_addr1          (ID_jalr_addr[0]),
    //     .jalr_addr2          (ID_jalr_addr[1]),
    //     .jalr_addr3          (ID_jalr_addr[2])
    // );


    wire bubble;
    EX_to_ID_bubble bubble_unit(
        .IDEX_rd_addr    ( IDEX_rd_addr_r),
        .IFID_rs1        (IFID_inst_r[19:15]),
        .IFID_rs2        (IFID_inst_r[24:20]),
        .IDEX_regWrite   (IDEX_RegWrite_r),
        .ID_branch_or_jalr( jalr|branch),
        .Bubble          ( bubble)
    );
//========================================================================================

//==== PC =============================
    //TODO

    //wire [31:0] pc_w_no_hazard;
    wire [31:0] pc_w;
    wire [31:0] pc_add4;
    reg  [31:0] pc_r;


//==== Connect the Circuit ============
    // PC
    assign pc_add4 = pc_r + 4 ;
    assign pc_w =   (hazard_stall|bubble) ? pc_r :
                    (jal|ID_branch_jump) ? ID_jal_or_branch_addr :
                    (jalr) ? ID_real_jalr_addr :
                    pc_add4;

    //wire    branch_flush; //judge if branch, if so, flush the following instrction
    //wire    j_flush;
    //assign j_flush = ( IDEX_jal_r | IDEX_jalr_r );

    // decide next pc
    //assign pc_w_no_hazard = ( IDEX_jal_r | branch_flush ) ? EXMEM_branch_or_jal_addr_w:
    //        ( IDEX_jalr_r )        ? EXMEM_jalr_addr_w:
    //        pc_r + 4;

    //assign pc_w = (hazard_stall)? pc_r: pc_w_no_hazard;
    // ID stage
    
    
    //TODO...
    assign wdata =  (MEMWB_MemToReg_r) ? MEMWB_rdata_r:
                    MEMWB_alu_out_r;

    
    wire [31:0] busY;
    register_file reg_file(
        .clk        ( clk )                 ,
        .rst_n      ( rst_n )               ,
        .WEN        ( MEMWB_RegWrite_r )    ,
        .RW         ( MEMWB_rd_addr_r )     ,
        .busW       ( wdata )               ,
        .RX         ( IFID_inst_r[19:15] )  ,
        .RY         ( IFID_inst_r[24:20] )  ,
        .busX       ( busX )       ,
        .busY       ( busY )
    );

    // forwarding ( only forward WB stage )
    assign IDEX_rdata1_w = (forward_a_id[1]) ? EXMEM_alu_out_r  :
                           (forward_a_id[0]) ? wdata            :
                                               busX             ; 
    assign IDEX_rdata2_w =  (forward_b_id[1])? EXMEM_alu_out_r  :
                            (forward_b_id[0])? wdata            :
                                               busY             ; 
    // check branch at EX stage
    
    ImmGen ig(
        .IR( IFID_inst_r[31:0] )            ,
        .Imm( IDEX_imm_w )
    );
    Control_unit ctrl_unit(
        .Opcode     ( IFID_inst_r[6:2] )    ,
        .Jal        ( jal )          ,
        .Jalr       ( jalr )         ,
        .Branch     ( branch )       ,
        .MemRead    ( memread )      ,
        .MemToReg   ( memtoreg )     ,
        .MemWrite   ( memwrite )     ,
        .ALUSrc     ( alusrc )       ,
        .RegWrite   ( regwrite )     ,
        .ALUop      ( aluop )
    );

    // control unit dealing hazard 
    // 0 when flush or bubble 

    // TODO...
    // assign IDEX_jal_w = ( j_flush | branch_flush | hazard_stall )?  1'b0: jal;
    // assign IDEX_jalr_w = ( j_flush | branch_flush | hazard_stall )? 1'b0: jalr;
    // assign IDEX_branch_w = ( j_flush | branch_flush | hazard_stall )? 1'b0: branch;
    assign IDEX_MemRead_w = (  hazard_stall |bubble)? 1'b0: memread;
    assign IDEX_MemToReg_w = ( hazard_stall |bubble)? 1'b0: memtoreg;
    assign IDEX_MemWrite_w = (  hazard_stall |bubble)? 1'b0: memwrite;
    assign IDEX_alusrc_w = (  hazard_stall |bubble)? 1'b0: alusrc;
    assign IDEX_RegWrite_w = (  hazard_stall |bubble)? 1'b0: regwrite;
    assign IDEX_aluop_w = (  hazard_stall |bubble)? 2'b0: aluop;


    assign IDEX_pc_addr_w = IFID_pc_addr_r;
    assign IDEX_func3_w   = IFID_inst_r[14:12];
    assign IDEX_func7_w   = IFID_inst_r[30];
    assign IDEX_rd_addr_w = IFID_inst_r[11:7];
    assign IDEX_rs1_w = IFID_inst_r[19:15];
    assign IDEX_rs2_w = IFID_inst_r[24:20];

    // EX stage

    // if jump, flush ex stage
    assign EXMEM_RegWrite_w =  IDEX_RegWrite_r;
    assign EXMEM_MemToReg_w =  IDEX_MemToReg_r;
    //assign EXMEM_jal_w      =  IDEX_jal_r;
    //assign EXMEM_jalr_w     =  IDEX_jalr_r;
    //assign EXMEM_branch_w   =  IDEX_branch_r;
    assign EXMEM_jump_w       =  IDEX_jump_r;
    assign EXMEM_MemRead_w  =  IDEX_MemRead_r;
    assign EXMEM_MemWrite_w =  IDEX_MemWrite_r;

    // assign EXMEM_jalr_addr_w = IDEX_imm_r + forwarding_x; //alu_in_x is forwarding data1, which is the address jalr will go
    //assign EXMEM_branch_or_jal_addr_w = IDEX_imm_r + IDEX_pc_addr_r;

    //assign EXMEM_jalr_addr_w = (forward_a_ex[1])? IDEX_imm_r + EXMEM_alu_out_r:
    //                           (forward_a_ex[0])? IDEX_imm_r + wdata          :
    //                                              IDEX_imm_r + IDEX_rdata1_r  ;


    assign EXMEM_wdata_w    = {forwarding_y[7:0], forwarding_y[15:8], forwarding_y[23:16], forwarding_y[31:24] };
    assign EXMEM_rd_addr_w  = IDEX_rd_addr_r;
    assign EXMEM_pc_add4_w  = IDEX_pc_addr_r + 4;


    wire  [31:0]    alu_in_y;
    assign forwarding_x = (forward_a_ex[1])  ? EXMEM_alu_out_r:
                          (forward_a_ex[0])  ? wdata          :
                                               IDEX_rdata1_r;
    assign alu_in_y = (IDEX_alusrc_r) ? IDEX_imm_r: forwarding_y;
    assign forwarding_y = (forward_b_ex[1])  ? EXMEM_alu_out_r:
                          (forward_b_ex[0])  ? wdata          :
                                               IDEX_rdata2_r;
                                               
    ALU alu(
        .ctrl   ( alu_ctrl )            ,
        .x      ( forwarding_x )        , 
        .y      ( alu_in_y )            , 
        .out    ( EXMEM_alu_out_w )     
        // .zero   ( EXMEM_alu_zero_w )
    );
    ALU_control alu_control(
        .Func7  ( IDEX_func7_r )        ,
        .Func3  ( IDEX_func3_r )        ,
        .OP     ( IDEX_aluop_r )        ,
        .ALUCtrl( alu_ctrl )
    );

    // MEM stage
    
    assign  MEMWB_RegWrite_w = EXMEM_RegWrite_r;
    assign  MEMWB_MemToReg_w = EXMEM_MemToReg_r;
    assign  MEMWB_alu_out_w  = (EXMEM_jump_r) ? EXMEM_pc_add4_r: EXMEM_alu_out_r;
    
    assign  DCACHE_ren  = EXMEM_MemRead_r;
    assign  DCACHE_wen  = EXMEM_MemWrite_r;
    assign  DCACHE_addr = EXMEM_alu_out_r[31:2];
    assign  DCACHE_wdata = EXMEM_wdata_r;
    assign  MEMWB_rdata_w = { DCACHE_rdata[7:0], DCACHE_rdata[15:8], DCACHE_rdata[23:16], DCACHE_rdata[31:24] };
    assign  MEMWB_rd_addr_w = EXMEM_rd_addr_r;
    assign  MEMWB_pc_add4_w = EXMEM_pc_add4_r;
    //assign  MEMWB_jal_w = EXMEM_jal_r;
    //assign  MEMWB_jalr_w = EXMEM_jalr_r;

    assign  ICACHE_addr = pc_r[31:2];

    // TODO...
    assign  IFID_inst_w =   (hazard_stall|bubble)?         IFID_inst_r  :  
                            (ID_branch_jump|IDEX_jump_w)      ? 32'h0000007f : // flush control unit (bubble)                   
    { ICACHE_rdata[7:0], ICACHE_rdata[15:8], ICACHE_rdata[23:16], ICACHE_rdata[31:24] };
  
    assign  IFID_pc_addr_w = (hazard_stall|bubble) ? IFID_pc_addr_r :pc_r;

    // proc_stall
    wire    proc_stall;
    assign  proc_stall = (DCACHE_stall|ICACHE_stall);

    // dealing hazard problem
    HAZARD_DETECTION_UNIT h(
        .IDEX_RT(IDEX_rd_addr_r),
        .IFID_RS(IFID_inst_r[19:15]),
        .IFID_RT(IFID_inst_r[24:20]),
        .MemRead(IDEX_MemRead_r),
        .stall(hazard_stall)
    );

    // forward data to ID / EX stage
    FORWARDING_UNIT ex(
        .EXMEM_RD(EXMEM_rd_addr_r),
        .IDEX_RS1(IDEX_rs1_r),
        .IDEX_RS2(IDEX_rs2_r),
        .IFID_RS1(IFID_inst_r[19:15]),
        .IFID_RS2(IFID_inst_r[24:20]),
        .MEMWB_RD(MEMWB_rd_addr_r),
        .EXMEM_RegWrite(EXMEM_RegWrite_r),
        .MEMWB_RegWrite(MEMWB_RegWrite_r),
        .FORWARD_A_ex(forward_a_ex),
        .FORWARD_B_ex(forward_b_ex),
        .FORWARD_A_id(forward_a_id),
        .FORWARD_B_id(forward_b_id)
    );

    always@(posedge clk or negedge rst_n) begin
        if(~rst_n) begin
            // reset the circuit
			pc_r 						<= 0;
            IFID_pc_addr_r              <= 0;
            IFID_inst_r                 <= 32'h00000013; //nop;
            IDEX_RegWrite_r             <= 0;
            IDEX_MemToReg_r             <= 0;
            //IDEX_jal_r                  <= 0;
            //IDEX_jalr_r                 <= 0;
            //IDEX_branch_r               <= 0;
            IDEX_jump_r                 <= 0;
            IDEX_MemRead_r              <= 0;
            IDEX_MemWrite_r             <= 0;
            IDEX_aluop_r                <= 0;
            IDEX_alusrc_r               <= 0;
            IDEX_pc_addr_r              <= 0;
            IDEX_rs1_r                  <= 0;
            IDEX_rs2_r                  <= 0;
            IDEX_rdata1_r               <= 0;
            IDEX_rdata2_r               <= 0;
            IDEX_imm_r                  <= 0;
            IDEX_func7_r                <= 0;
            IDEX_func3_r                <= 0;
            IDEX_rd_addr_r              <= 0;
            EXMEM_RegWrite_r            <= 0;
            EXMEM_MemToReg_r            <= 0;
            //EXMEM_jal_r                 <= 0;
            //EXMEM_jalr_r                <= 0;
            //EXMEM_branch_r              <= 0;
            EXMEM_jump_r                <= 0;
            EXMEM_MemRead_r             <= 0;
            EXMEM_MemWrite_r            <= 0;
            // EXMEM_jalr_addr_r           <= 0;
            // EXMEM_branch_or_jal_addr_r  <= 0;
            EXMEM_alu_out_r             <= 0;
            // EXMEM_alu_zero_r            <= 0;
            EXMEM_wdata_r               <= 0;
            EXMEM_rd_addr_r             <= 0;
            EXMEM_pc_add4_r             <= 0;
            MEMWB_RegWrite_r            <= 0;
            MEMWB_MemToReg_r            <= 0;
            MEMWB_alu_out_r             <= 0;
            MEMWB_rdata_r               <= 0;
            MEMWB_rd_addr_r             <= 0;
            MEMWB_pc_add4_r             <= 0;
            //MEMWB_jal_r                 <= 0;
            //MEMWB_jalr_r                <= 0;
        end
        else begin
            if(~proc_stall) begin
				pc_r					<= pc_w;
                IFID_pc_addr_r          <= IFID_pc_addr_w;
                IFID_inst_r             <= IFID_inst_w;
                IDEX_RegWrite_r         <= IDEX_RegWrite_w;
                IDEX_MemToReg_r         <= IDEX_MemToReg_w;
                //IDEX_jal_r              <= IDEX_jal_w;
                //IDEX_jalr_r             <= IDEX_jalr_w;
                //IDEX_branch_r           <= IDEX_branch_w;
                IDEX_jump_r             <= IDEX_jump_w;
                IDEX_MemRead_r          <= IDEX_MemRead_w;
                IDEX_MemWrite_r         <= IDEX_MemWrite_w;
                IDEX_aluop_r            <= IDEX_aluop_w;
                IDEX_alusrc_r           <= IDEX_alusrc_w;
                IDEX_pc_addr_r          <= IDEX_pc_addr_w;
                IDEX_rs2_r              <= IDEX_rs2_w;
                IDEX_rs1_r              <= IDEX_rs1_w;
                IDEX_rdata1_r           <= IDEX_rdata1_w;
                IDEX_rdata2_r           <= IDEX_rdata2_w;
                IDEX_imm_r              <= IDEX_imm_w;
                IDEX_func7_r            <= IDEX_func7_w;
                IDEX_func3_r            <= IDEX_func3_w;
                IDEX_rd_addr_r          <= IDEX_rd_addr_w;
                EXMEM_RegWrite_r        <= EXMEM_RegWrite_w;
                EXMEM_MemToReg_r        <= EXMEM_MemToReg_w;
                //EXMEM_jal_r             <= EXMEM_jal_w;
                //EXMEM_jalr_r            <= EXMEM_jalr_w;
                //EXMEM_branch_r          <= EXMEM_branch_w;
                EXMEM_jump_r            <= EXMEM_jump_w;
                EXMEM_MemRead_r         <= EXMEM_MemRead_w;
                EXMEM_MemWrite_r        <= EXMEM_MemWrite_w;
                // EXMEM_jalr_addr_r       <= EXMEM_jalr_addr_w;
                // EXMEM_branch_or_jal_addr_r  <= EXMEM_branch_or_jal_addr_w;
                EXMEM_alu_out_r             <= EXMEM_alu_out_w;
                // EXMEM_alu_zero_r            <= EXMEM_alu_zero_w;
                EXMEM_wdata_r               <= EXMEM_wdata_w;
                EXMEM_rd_addr_r             <= EXMEM_rd_addr_w;
                EXMEM_pc_add4_r             <= EXMEM_pc_add4_w;
                MEMWB_RegWrite_r            <= MEMWB_RegWrite_w;
                MEMWB_MemToReg_r            <= MEMWB_MemToReg_w;
                MEMWB_alu_out_r             <= MEMWB_alu_out_w;
                MEMWB_rdata_r               <= MEMWB_rdata_w;
                MEMWB_rd_addr_r             <= MEMWB_rd_addr_w;
                MEMWB_pc_add4_r             <= MEMWB_pc_add4_w;
                //MEMWB_jal_r                 <= MEMWB_jal_w;
                //MEMWB_jalr_r                <= MEMWB_jalr_w;
            end
        end
    end
endmodule

//===================================================
//=                 Register File                   =
//===================================================

module register_file(
    clk  ,
    rst_n,
    WEN  ,
    RW   ,
    busW ,
    RX   ,
    RY   ,
    busX ,
    busY
);
input        clk, WEN, rst_n;
input  [4:0] RW, RX, RY; // 5bit read from instruction code
input  [31:0] busW;
output [31:0] busX, busY;
    
// write your design here, you can delcare your own wires and regs. 
// The code below is just an eaxmple template
reg [31:0] r_w [0:31];
reg [31:0] r_r [0:31];
reg [31:0] busX, busY;
integer i;
    
always@(*) begin
    for(i=0; i<32; i = i+1) begin
        r_w[i] = r_r[i];
    end
        
    if (WEN) begin
        case (RW)
            5'd0 : r_w [0] = 32'b0;
            5'd1 : r_w [1] = busW;
            5'd2 : r_w [2] = busW;
            5'd3 : r_w [3] = busW;
            5'd4 : r_w [4] = busW;
            5'd5 : r_w [5] = busW;
            5'd6 : r_w [6] = busW;
            5'd7 : r_w [7] = busW;
            5'd8 : r_w [8] = busW;
            5'd9 : r_w [9] = busW;
            5'd10 : r_w [10] = busW;
            5'd11 : r_w [11] = busW;
            5'd12 : r_w [12] = busW;
            5'd13 : r_w [13] = busW;
            5'd14 : r_w [14] = busW;
            5'd15 : r_w [15] = busW;
            5'd16 : r_w [16] = busW;
            5'd17 : r_w [17] = busW;
            5'd18 : r_w [18] = busW;
            5'd19 : r_w [19] = busW;
            5'd20 : r_w [20] = busW;
            5'd21 : r_w [21] = busW;
            5'd22 : r_w [22] = busW;
            5'd23 : r_w [23] = busW;
            5'd24 : r_w [24] = busW;
            5'd25 : r_w [25] = busW;
            5'd26 : r_w [26] = busW;
            5'd27 : r_w [27] = busW;
            5'd28 : r_w [28] = busW;
            5'd29 : r_w [29] = busW;
            5'd30 : r_w [30] = busW;
            5'd31 : r_w [31] = busW;
        endcase
    end
    else begin
        for(i=0; i<32; i = i+1) begin
            r_w[i] = r_r[i];
        end
    end
    case (RX)
        5'd0 : busX = 32'b0;
        5'd1 : busX = r_r[1];
        5'd2 : busX = r_r[2];
        5'd3 : busX = r_r[3];
        5'd4 : busX = r_r[4];
        5'd5 : busX = r_r[5];
        5'd6 : busX = r_r[6];
        5'd7 : busX = r_r[7];
        5'd8 : busX = r_r[8];
        5'd9 : busX = r_r[9];
        5'd10 : busX = r_r[10];
        5'd11 : busX = r_r[11];
        5'd12 : busX = r_r[12];
        5'd13 : busX = r_r[13];
        5'd14 : busX = r_r[14];
        5'd15 : busX = r_r[15];
        5'd16 : busX = r_r[16];
        5'd17 : busX = r_r[17];
        5'd18 : busX = r_r[18];
        5'd19 : busX = r_r[19];
        5'd20 : busX = r_r[20];
        5'd21 : busX = r_r[21];
        5'd22 : busX = r_r[22];
        5'd23 : busX = r_r[23];
        5'd24 : busX = r_r[24];
        5'd25 : busX = r_r[25];
        5'd26 : busX = r_r[26];
        5'd27 : busX = r_r[27];
        5'd28 : busX = r_r[28];
        5'd29 : busX = r_r[29];
        5'd30 : busX = r_r[30];
        5'd31 : busX = r_r[31];
    endcase

    case (RY)
        5'd0 : busY = 32'b0;
        5'd1 : busY = r_r[1];
        5'd2 : busY = r_r[2];
        5'd3 : busY = r_r[3];
        5'd4 : busY = r_r[4];
        5'd5 : busY = r_r[5];
        5'd6 : busY = r_r[6];
        5'd7 : busY = r_r[7];
        5'd8 : busY = r_r[8];
        5'd9 : busY = r_r[9];
        5'd10 : busY = r_r[10];
        5'd11 : busY = r_r[11];
        5'd12 : busY = r_r[12];
        5'd13 : busY = r_r[13];
        5'd14 : busY = r_r[14];
        5'd15 : busY = r_r[15];
        5'd16 : busY = r_r[16];
        5'd17 : busY = r_r[17];
        5'd18 : busY = r_r[18];
        5'd19 : busY = r_r[19];
        5'd20 : busY = r_r[20];
        5'd21 : busY = r_r[21];
        5'd22 : busY = r_r[22];
        5'd23 : busY = r_r[23];
        5'd24 : busY = r_r[24];
        5'd25 : busY = r_r[25];
        5'd26 : busY = r_r[26];
        5'd27 : busY = r_r[27];
        5'd28 : busY = r_r[28];
        5'd29 : busY = r_r[29];
        5'd30 : busY = r_r[30];
        5'd31 : busY = r_r[31];
    endcase
end

always@(posedge clk or negedge rst_n) begin
    if(!rst_n) begin
        for(i=0; i<32; i=i+1) begin
            r_r[i] <= 32'b0;
        end
    end
    else begin
        r_r [0] <= 32'b0;
        for(i=1;i<32;i=i+1) begin
            r_r [i] <= r_w[i];
        end
    end
end	

endmodule

//==================================
//=             ALU                =
//==================================
module ALU (
    ctrl,
    x,
    y,
    out
    // zero
);
    input   [3:0]  ctrl;    // the contral signal which decide the operation of alu
    input   [31:0] x, y;    // 32 bit input data from register file or instruction[15:0]
    output  [31:0] out;     // 32 bit output
    // output         zero;    // input for branch control              

    reg [31:0] out;
    wire [4:0] shamt; // shift amount unsigned 5-bit

    assign shamt = y[4:0];

    always@(*) begin
    case ( ctrl )
        4'b0000: begin out = x + y; end                         //ADD
        4'b1000: begin out = {x[31], x} + ~{y[31],y} + 1; end   //SUB
        4'b0010: begin
            out = ({x[31], x} + ~{y[31],y} + 1) >>31;           //SLT
        end                                                          
        4'b0100: begin out = x ^ y; end                         //XOR
        4'b0110: begin out = x | y; end                         //OR
        4'b0111: begin out = x & y; end                         //AND
        4'b0001: begin out = x << shamt; end                    //SLL
        4'b0101: begin out = x >> shamt; end                    //SRL
        4'b1101: begin out = $signed(x) >>> shamt; end          //SRA
        default: begin
            out = 32'b0;
        end 
    endcase

    end

endmodule

//===============================================
//=             Immediate Generator             =
//===============================================
module ImmGen(
    IR,
    Imm
);
    input   [31:0]  IR;     // instruction from instruction memory
    output  [31:0]  Imm;    
    reg     [31:0]  Imm;

    always@(*) begin
        case (IR[6:2])
            5'b00000:   Imm = {{21{IR[31]}},IR[30:25],IR[24:21],IR[20]};                     // lw I-type
            5'b01000:   Imm = {{21{IR[31]}},IR[30:25],IR[11:8],IR[7]};                       // sw S-type
            5'b11000:   Imm = {{20{IR[31]}}, IR[7], IR[30:25], IR[11:8], 1'b0};              // beq B-type
            5'b11011:   Imm = {{12{IR[31]}}, IR[19:12], IR[20], IR[30:25], IR[24:21], 1'b0}; // jal J-type
            5'b11001:   Imm = {{21{IR[31]}},IR[30:25],IR[24:21],IR[20]};                     // jalr I-type
            5'b00100:   Imm = { {20{IR[31]}},IR[31:20]};                                     // I-type
            default:    Imm = 32'b0;
        endcase
    end
endmodule

//=======================================
//              ALU Control             =
//=======================================
module ALU_control(
    Func7,
    Func3,
    OP,
    ALUCtrl
);
    input           Func7;
    input   [2:0]   Func3;
    input   [1:0]   OP;
    output  [3:0]   ALUCtrl;

    reg [3:0]       ALUCtrl;
    wire            isSRA;

    assign isSRA = ({Func7, Func3[2:0]} == 4'b1101) ? 1'b1 : 1'b0;

    always@(*) begin
        case(OP)
            2'b00: begin
                //ALUop = 00, JALR, LW, SW
                ALUCtrl = 4'b0000;
            end
            2'b01: begin
                //ALUop = 01, BEQ, BNE
                // if(Func3[0]) begin
                //     //BNE
                //     ALUCtrl = 4'b0011;
                // end
                // else begin
                //     //BEQ
                //     ALUCtrl = 4'b1011;
                // end
                ALUCtrl = 4'b0011;
            end
            2'b10: begin
                ALUCtrl = {Func7, Func3[2:0]};
            end
            2'b11: begin
                if(isSRA) begin
                    ALUCtrl = {Func7, Func3[2:0]};
                end
                else begin
                    ALUCtrl = {1'b0, Func3[2:0]};
                end
            end
        endcase
    end
    
endmodule

//========================
//=    Control Unit      =
//========================
module Control_unit(
    Opcode,
    Jal,
    Jalr,
    Branch,
    MemRead,
    MemToReg,
    MemWrite,
    ALUSrc,
    RegWrite,
    ALUop
);
    input   [4:0]   Opcode;
    output  reg     Jal, Jalr, Branch, MemRead, MemToReg, MemWrite, ALUSrc, RegWrite;
    output  reg [1:0] ALUop;
    always@(*) begin
        case ( Opcode )
            5'b01100:   begin
                        // R-type
                            Jal         = 1'b0;
                            Jalr        = 1'b0;
                            Branch      = 1'b0;
                            MemRead     = 1'b0;
                            MemToReg    = 1'b0;
                            MemWrite    = 1'b0;
                            ALUSrc      = 1'b0;
                            RegWrite    = 1'b1;
                            ALUop       = 2'b10;
                        end 
            5'b00000:   begin
                        // lw
                            Jal         = 1'b0;
                            Jalr        = 1'b0;
                            Branch      = 1'b0;
                            MemRead     = 1'b1;
                            MemToReg    = 1'b1;
                            MemWrite    = 1'b0;
                            ALUSrc      = 1'b1;
                            RegWrite    = 1'b1;
                            ALUop       = 2'b00;
                        end 
            5'b01000:   begin
                        // sw
                            Jal         = 1'b0;
                            Jalr        = 1'b0;
                            Branch      = 1'b0;
                            MemRead     = 1'b0;
                            MemToReg    = 1'b0;
                            MemWrite    = 1'b1;
                            ALUSrc      = 1'b1;
                            RegWrite    = 1'b0;
                            ALUop       = 2'b00;
                        end 
            5'b11000:   begin
                        // beq
                            Jal         = 1'b0;
                            Jalr        = 1'b0;
                            Branch      = 1'b1;
                            MemRead     = 1'b0;
                            MemToReg    = 1'b0;
                            MemWrite    = 1'b0;
                            ALUSrc      = 1'b0;
                            RegWrite    = 1'b0;
                            ALUop       = 2'b01;
                        end 
            5'b11011:   begin
                        // jal
                            Jal         = 1'b1;
                            Jalr        = 1'b0;
                            Branch      = 1'b0;
                            MemRead     = 1'b0;
                            MemToReg    = 1'b0;
                            MemWrite    = 1'b0;
                            ALUSrc      = 1'b0;
                            RegWrite    = 1'b1;
                            ALUop       = 2'b00;
                        end 
            5'b11001:   begin
                        // jalr
                            Jal         = 1'b0;
                            Jalr        = 1'b1;
                            Branch      = 1'b0;
                            MemRead     = 1'b0;
                            MemToReg    = 1'b0;
                            MemWrite    = 1'b0;
                            ALUSrc      = 1'b0;
                            RegWrite    = 1'b1;
                            ALUop       = 2'b00;
                        end
            5'b00100:   begin
                        // I-type
                            Jal         = 1'b0;
                            Jalr        = 1'b0;
                            Branch      = 1'b0;
                            MemRead     = 1'b0;
                            MemToReg    = 1'b0;
                            MemWrite    = 1'b0;
                            ALUSrc      = 1'b1;
                            RegWrite    = 1'b1;
                            ALUop       = 2'b11;
                        end              
            default:    begin
                            Jal         = 1'b0;
                            Jalr        = 1'b0;
                            Branch      = 1'b0;
                            MemRead     = 1'b0;
                            MemToReg    = 1'b0;
                            MemWrite    = 1'b0;
                            ALUSrc      = 1'b0;
                            RegWrite    = 1'b0;
                            ALUop       = 2'b00;
                        end 
        endcase
    end        

endmodule


 
// module ID_jump(
//     pc,
//     offset,
//     rdata1,
//     forward_data1,
//     forward_data2,
//     jal_or_branch_addr,
//     jalr_addr1,
//     jalr_addr2,
//     jalr_addr3
// );
//     input  [31:0] pc, offset, rdata1, forward_data1, forward_data2;
//     output [31:0] jal_or_branch_addr, jalr_addr1, jalr_addr2, jalr_addr3;

//     assign jal_or_branch_addr   = pc + offset;
//     assign jalr_addr1           = rdata1 + offset;
//     assign jalr_addr2           = forward_data1 + offset;
//     assign jalr_addr3           = forward_data2 + offset;


// endmodule

module EX_to_ID_bubble(
    IDEX_rd_addr,
    IFID_rs1,
    IFID_rs2,
    IDEX_regWrite,
    ID_branch_or_jalr,
    Bubble
);
    input [4:0]IDEX_rd_addr;
    input [4:0]IFID_rs1;
    input [4:0]IFID_rs2;
    input IDEX_regWrite;
    input ID_branch_or_jalr;
    output Bubble;

    wire FLAG1, FLAG2;
    assign FLAG1 = (IFID_rs1==IDEX_rd_addr)&&(IFID_rs1!=5'b0);
    assign FLAG2 = (IFID_rs2==IDEX_rd_addr)&&(IFID_rs2!=5'b0);

    assign Bubble = ((IDEX_regWrite && (FLAG1|FLAG2)) && ID_branch_or_jalr);

endmodule

module HAZARD_DETECTION_UNIT(
	input [4:0] IDEX_RT,
	input [4:0] IFID_RS,
	input [4:0] IFID_RT,
	input       MemRead,
	output  reg    stall
	);
	
	/*
	if (stall) begin
		PCWrite = 1'b0;
		IF_ID_Write = 1'b0;
		Ctrl_Sel = 1'b0;
	end else begin
		PCWrite = 1'b1;
		IF_ID_Write = 1'b1;
		Ctrl_Sel = 1'b1;
	end
	*/
	//reg stall;
	always@(*) begin
		if (MemRead && ( (IDEX_RT == IFID_RS) | (IDEX_RT == IFID_RT) ) ) begin
			stall = 1'b1;
		end else begin
			stall = 1'b0;
		end
	end

endmodule

module FORWARDING_UNIT(
	input  [4:0] EXMEM_RD,
	input  [4:0] IDEX_RS1,
	input  [4:0] IDEX_RS2,
	input  [4:0] IFID_RS1,
	input  [4:0] IFID_RS2,	
	input  [4:0] MEMWB_RD,
	input        EXMEM_RegWrite,
	input        MEMWB_RegWrite,
	output reg [1:0] FORWARD_A_ex,
	output reg [1:0] FORWARD_B_ex,
	output reg [1:0] FORWARD_A_id,
	output reg [1:0] FORWARD_B_id
	);
	
	wire FLAG1, FLAG2, FLAG3, FLAG4;
	wire FLAG5, FLAG6, FLAG7, FLAG8;
	wire FLAG_RS1_ZERO, FLAG_RS2_ZERO;


	assign FLAG1 = ~|(EXMEM_RD ^ IDEX_RS1);
	assign FLAG2 = ~|(MEMWB_RD ^ IDEX_RS1);
	assign FLAG3 = ~|(EXMEM_RD ^ IDEX_RS2);
	assign FLAG4 = ~|(MEMWB_RD ^ IDEX_RS2);

	assign FLAG5 = ~|(EXMEM_RD ^ IFID_RS1);
	assign FLAG6 = ~|(MEMWB_RD ^ IFID_RS1);
	assign FLAG7 = ~|(EXMEM_RD ^ IFID_RS2);
	assign FLAG8 = ~|(MEMWB_RD ^ IFID_RS2);

	assign FLAG_RS1_ZERO = (|(IFID_RS1))? 1'b1: 1'b0;
	assign FLAG_RS2_ZERO = (|(IFID_RS2))? 1'b1: 1'b0;

	assign FLAG_RS1_ZERO_EX = (|(IDEX_RS1))? 1'b1: 1'b0;
	assign FLAG_RS2_ZERO_EX = (|(IDEX_RS2))? 1'b1: 1'b0;

	always@(*) begin
		FORWARD_A_ex[1] = (EXMEM_RegWrite && FLAG1 && (FLAG_RS1_ZERO_EX));
		FORWARD_A_ex[0] = (MEMWB_RegWrite && FLAG2 && (FLAG_RS1_ZERO_EX));

		FORWARD_B_ex[1] = (EXMEM_RegWrite && FLAG3 && (FLAG_RS2_ZERO_EX));
		FORWARD_B_ex[0] = (MEMWB_RegWrite && FLAG4 && (FLAG_RS2_ZERO_EX));
		
		
		FORWARD_A_id[1] = (EXMEM_RegWrite && FLAG5 && (FLAG_RS1_ZERO));
		FORWARD_A_id[0] = (MEMWB_RegWrite && FLAG6 && (FLAG_RS1_ZERO));
		

		FORWARD_B_id[1] = (EXMEM_RegWrite && FLAG7 && (FLAG_RS2_ZERO));
		FORWARD_B_id[0] = (MEMWB_RegWrite && FLAG8 && (FLAG_RS2_ZERO));
	end

endmodule


module cache_read_only(
    clk,
    proc_reset,
    proc_read,
    proc_write,
    proc_addr,
    proc_rdata,
    proc_wdata,
    proc_stall,
    mem_read,
    mem_write,
    mem_addr,
    mem_rdata,
    mem_wdata,
    mem_ready
);

//==== parameters definition ==============================
    // for FSM state...
    localparam  START       = 2'b00;
    localparam  ALLOCATE    = 2'b01;
    localparam  BUFFER      = 2'b10;

    // for loop
    integer i;


//==== input/output definition ============================
    input          clk;
    // processor interface
    input          proc_reset;
    input          proc_read, proc_write;
    input   [29:0] proc_addr;
    input   [31:0] proc_wdata;
    output         proc_stall;
    output  [31:0] proc_rdata;
    // memory interface
    input  [127:0] mem_rdata;
    input          mem_ready;
    output  reg    mem_read, mem_write;
    output  reg [27:0] mem_addr;
    output [127:0] mem_wdata;

//==== wire/reg definition ================================
    //for storage
    reg         valid_w[0:7];
    reg         valid_r[0:7];
    //reg         dirty_w[0:7];
    //reg         dirty_r[0:7];
    reg  [24:0] tag_w  [0:7];
    reg  [24:0] tag_r  [0:7];
    reg  [31:0] word_w[0:31];
    reg  [31:0] word_r[0:31];

    // for FSM
    reg  [1:0]  state_nxt;
    reg  [1:0]  state;

    // hit or miss
    wire     hit_or_miss; // 1 for hit, 0 for miss
    //reg     dirty;

    // for circuit output
    reg [31:0]  rdata;
    reg         stall;
    reg [127:0] wdata;

    // for buffer state
    reg  [127:0]  wdata_buf_w;
    reg  [127:0]  wdata_buf_r;
    //reg  [27:0]   mem_addr_buf_w, mem_addr_buf_r;


//==== Finite State Machine ===============================
always@( posedge clk) begin
    if( proc_reset ) begin
        state <= START;
    end
    else begin
        state <= state_nxt;
    end
end
//==== next state logic =====================================
always@(*) begin
    case ( state )
        START:
            begin 
                if ((proc_read|proc_write)) begin
                    if( hit_or_miss  ) begin
                        // hit!!
                        state_nxt = START;
                    end
                    else begin
                        state_nxt = ALLOCATE;
                    end
                end
                else begin
                    state_nxt = state;
                end
            end
        ALLOCATE:
            begin
                if( mem_ready ) begin
                    state_nxt = BUFFER;
                end
                else begin
                    state_nxt = ALLOCATE;
                end
            end
        BUFFER:
            begin
                state_nxt = START;
            end
        default:
            begin
                state_nxt = state;
            end
    endcase
end
//==== combinational circuit ==============================

assign proc_rdata = word_r[proc_addr[4:0]];
assign proc_stall = stall;
assign mem_wdata  = 0;
assign hit_or_miss = ({valid_r[proc_addr[4:2]],tag_r[proc_addr[4:2]]} == {1'b1,proc_addr[29:5]});

always@(*) begin
    //==== Default value ==================================
    //dirty =         1'b0;
    //rdata =         32'b0;
    stall =         1'b0;
    wdata =         128'b0;
    mem_read =      0;
    mem_write =     0;
    mem_addr   = proc_addr[29:2];
    wdata_buf_w = mem_rdata;
    //mem_addr_buf_w = mem_addr_buf_r;
    for (i=0;i<8;i=i+1) begin
        valid_w[i] = valid_r[i]; 
        //dirty_w[i] = dirty_r[i]; 
        tag_w[i]   = tag_r[i]; 
    end
    for (i=0;i<32;i=i+1) begin
        word_w[i]  = word_r[i]; 
    end
    //===========================
    case ( state ) 
        START:
            begin
                if( hit_or_miss) begin
                    // hit
                    stall = 1'b0;
                    
                end
                else begin
                    // miss
                    stall = 1'b1;       
                                 
                end
            end
        ALLOCATE:
            begin
                stall = 1'b1;
                case ( proc_addr[4:2] ) 
                        3'd0: tag_w[0] = proc_addr[29:5];
                        3'd1: tag_w[1] = proc_addr[29:5];
                        3'd2: tag_w[2] = proc_addr[29:5];
                        3'd3: tag_w[3] = proc_addr[29:5];
                        3'd4: tag_w[4] = proc_addr[29:5];
                        3'd5: tag_w[5] = proc_addr[29:5];
                        3'd6: tag_w[6] = proc_addr[29:5];
                        3'd7: tag_w[7] = proc_addr[29:5];
                endcase
                valid_w[proc_addr[4:2]] = 1'b1;
                
                    mem_read =  1'b1;
                    mem_write = 1'b0;
            end
        BUFFER:
            begin
                stall = 1'b1;
                {{word_w[{proc_addr[4:2], 2'b11}]}, {word_w[{proc_addr[4:2], 2'b10}]},
                {word_w[{proc_addr[4:2], 2'b01}]}, {word_w[{proc_addr[4:2], 2'b00}]}} = wdata_buf_r;
            end
        default:
            begin
                stall = 1'b1;
                mem_read = 1'b0;
                mem_write = 1'b0;
            end
    endcase
end
//==== sequential circuit =================================
always@( posedge clk or posedge proc_reset) begin
    if( proc_reset ) begin
        for (i=0;i<8;i=i+1) begin
            valid_r[i] <= 1'b0; // reset valid bit
            //dirty_r[i] <= 1'b0; // reset dirty bit
            tag_r[i]   <= 25'b0; // reset tag
        end
        for (i=0;i<32;i=i+1) begin
            word_r[i]  <= 32'b0; // reset words
        end
        wdata_buf_r <= 0;
        //mem_addr_buf_r <=0;
    end
    else begin
        for (i=0;i<8;i=i+1) begin
            valid_r[i] <= valid_w[i]; // reset valid bit
            //dirty_r[i] <= dirty_w[i]; // reset dirty bit
            tag_r[i]   <= tag_w[i]; // reset tag
        end
        for (i=0;i<32;i=i+1) begin
            word_r[i]  <= word_w[i]; // reset words
        end
        wdata_buf_r <= wdata_buf_w;
       // mem_addr_buf_r <= mem_addr_buf_w;
    end
end

endmodule

module cache(
    clk,
    proc_reset,
    proc_read,
    proc_write,
    proc_addr,
    proc_rdata,
    proc_wdata,
    proc_stall,
    mem_read,
    mem_write,
    mem_addr,
    mem_rdata,
    mem_wdata,
    mem_ready
);

//==== parameters definition ==============================
    // for FSM state...
    localparam  START       = 2'b00;
    localparam  ALLOCATE    = 2'b01;
    localparam  WRITE_BACK  = 2'b10;
    localparam  BUFFER      = 2'b11;

    // for loop
    integer i;


//==== input/output definition ============================
    input          clk;
    // processor interface
    input          proc_reset;
    input          proc_read, proc_write;
    input   [29:0] proc_addr;
    input   [31:0] proc_wdata;
    output         proc_stall;
    output  [31:0] proc_rdata;
    // memory interface
    input  [127:0] mem_rdata;
    input          mem_ready;
    output  reg    mem_read, mem_write;
    output [27:0]  mem_addr;
    output [127:0] mem_wdata;

//==== wire/reg definition ================================
    //for storage
    reg         valid_w[0:7];
    reg         valid_r[0:7];
    reg         dirty_w[0:7];
    reg         dirty_r[0:7];
    reg  [24:0] tag_w  [0:7];
    reg  [24:0] tag_r  [0:7];
    reg  [31:0] word_w[0:31];
    reg  [31:0] word_r[0:31];

    // for FSM
    reg  [1:0]  state_nxt;
    reg  [1:0]  state;

    // hit or miss
    wire     hit_or_miss; // 1 for hit, 0 for miss
    reg     dirty;

    // for circuit output
    reg [31:0]  rdata;
    reg         stall;
    reg [127:0] wdata;

    // for buffer state
    reg  [127:0]  wdata_buf_w;
    reg  [127:0]  wdata_buf_r;
    reg  [27:0]   mem_addr_buf_w, mem_addr_buf_r;
    wire [27:0]   mem_waddr;

    assign mem_waddr = { tag_r[proc_addr[4:2]], proc_addr[4:2] };


//==== Finite State Machine ===============================
always@( posedge clk) begin
    if( proc_reset ) begin
        state <= START;
    end
    else begin
        state <= state_nxt;
    end
end
//==== next state logic =====================================
always@(*) begin
    case ( state )
        START:
            begin 
                if ((proc_read|proc_write)) begin
                    if( hit_or_miss ) begin
                        // hit!!
                        state_nxt = START;
                    end
                    else begin
                        if( dirty ) begin
                            state_nxt = WRITE_BACK;
                        end
                        else begin
                            state_nxt = ALLOCATE;
                        end
                    end
                end
                else begin
                    state_nxt = state;
                end
            end
        ALLOCATE:
            begin
                if( mem_ready ) begin
                    state_nxt = BUFFER;
                end
                else begin
                    state_nxt = ALLOCATE;
                end
            end
        WRITE_BACK:
            begin
                if( mem_ready ) begin
                    state_nxt = ALLOCATE;
                end
                else begin
                    state_nxt = WRITE_BACK;
                end
            end
        BUFFER:
            begin
                    state_nxt = START;
            end
    endcase
end
//==== combinational circuit ==============================

assign proc_rdata = word_r[proc_addr[4:0]];
assign proc_stall = stall;
assign mem_wdata  = wdata;
assign hit_or_miss = ({valid_r[proc_addr[4:2]],tag_r[proc_addr[4:2]]} == {1'b1,proc_addr[29:5]});
assign mem_addr = mem_addr_buf_r;

always@(*) begin
    //==== Default value ==================================
    dirty =         1'b0;
    //rdata =         32'b0;
    stall =         1'b0;
    wdata =         128'b0;
    mem_read =      0;
    mem_write =     0;
    wdata_buf_w = mem_rdata;
    mem_addr_buf_w = mem_addr_buf_r;
    for (i=0;i<8;i=i+1) begin
        valid_w[i] = valid_r[i]; 
        dirty_w[i] = dirty_r[i]; 
        tag_w[i]   = tag_r[i]; 
    end
    for (i=0;i<32;i=i+1) begin
        word_w[i]  = word_r[i]; 
    end
    //===========================
    case ( state ) 
        START:
            begin
                dirty = dirty_r[proc_addr[4:2]];
                if( hit_or_miss) begin
                    // hit
                    stall = 1'b0;
                    //rdata = word_r[proc_addr[4:0]];
                    if( proc_write ) begin
                        //word_w[proc_addr[4:0]] = proc_wdata;
                        case(proc_addr[4:0])
                            5'd0: word_w[0] = proc_wdata;
                            5'd1: word_w[1] = proc_wdata;
                            5'd2: word_w[2] = proc_wdata;
                            5'd3: word_w[3] = proc_wdata;
                            5'd4: word_w[4] = proc_wdata;
                            5'd5: word_w[5] = proc_wdata;
                            5'd6: word_w[6] = proc_wdata;
                            5'd7: word_w[7] = proc_wdata;
                            5'd8: word_w[8] = proc_wdata;
                            5'd9: word_w[9] = proc_wdata;
                            5'd10: word_w[10] = proc_wdata;
                            5'd11: word_w[11] = proc_wdata;
                            5'd12: word_w[12] = proc_wdata;
                            5'd13: word_w[13] = proc_wdata;
                            5'd14: word_w[14] = proc_wdata;
                            5'd15: word_w[15] = proc_wdata;
                            5'd16: word_w[16] = proc_wdata;
                            5'd17: word_w[17] = proc_wdata;
                            5'd18: word_w[18] = proc_wdata;
                            5'd19: word_w[19] = proc_wdata;
                            5'd20: word_w[20] = proc_wdata;
                            5'd21: word_w[21] = proc_wdata;
                            5'd22: word_w[22] = proc_wdata;
                            5'd23: word_w[23] = proc_wdata;
                            5'd24: word_w[24] = proc_wdata;
                            5'd25: word_w[25] = proc_wdata;
                            5'd26: word_w[26] = proc_wdata;
                            5'd27: word_w[27] = proc_wdata;
                            5'd28: word_w[28] = proc_wdata;
                            5'd29: word_w[29] = proc_wdata;
                            5'd30: word_w[30] = proc_wdata;
                            5'd31: word_w[31] = proc_wdata;
                        endcase
                        //dirty_w[proc_addr[4:2]] = 1'b1;
                        case(proc_addr[4:2])
                            3'd0: dirty_w[0] = 1'b1;
                            3'd1: dirty_w[1] = 1'b1;
                            3'd2: dirty_w[2] = 1'b1;
                            3'd3: dirty_w[3] = 1'b1;
                            3'd4: dirty_w[4] = 1'b1;
                            3'd5: dirty_w[5] = 1'b1;
                            3'd6: dirty_w[6] = 1'b1;
                            3'd7: dirty_w[7] = 1'b1;
                        endcase
                    end
                end
                else begin
                    // miss
                    if((proc_read|proc_write)) begin
                        stall = 1'b1;
                        if(dirty) mem_write = 1'b1;
                        else      mem_read  = 1'b1;
                    end
                    else begin
                        stall = 1'b0;
                    end

                end
            end
        ALLOCATE:
            begin
                stall = 1'b1;
                case ( proc_addr[4:2] ) 
                        3'd0: tag_w[0] = proc_addr[29:5];
                        3'd1: tag_w[1] = proc_addr[29:5];
                        3'd2: tag_w[2] = proc_addr[29:5];
                        3'd3: tag_w[3] = proc_addr[29:5];
                        3'd4: tag_w[4] = proc_addr[29:5];
                        3'd5: tag_w[5] = proc_addr[29:5];
                        3'd6: tag_w[6] = proc_addr[29:5];
                        3'd7: tag_w[7] = proc_addr[29:5];
                endcase
                valid_w[proc_addr[4:2]] = 1'b1;
                dirty_w[proc_addr[4:2]] = 1'b0;
                mem_addr_buf_w = proc_addr[29:2];
                mem_read =  1'b1;
                mem_write = 1'b0;
            end
        WRITE_BACK:
            begin
                stall = 1'b1;
                mem_addr_buf_w = mem_waddr;
                wdata = { {word_r[{proc_addr[4:2], 2'b11}]}, {word_r[{proc_addr[4:2], 2'b10}]},
                                      {word_r[{proc_addr[4:2], 2'b01}]}, {word_r[{proc_addr[4:2], 2'b00}]} };
                mem_write = 1'b1;
                mem_read = 1'b0;
            end
        BUFFER:
            begin
                stall = 1'b1;
                {{word_w[{proc_addr[4:2], 2'b11}]}, {word_w[{proc_addr[4:2], 2'b10}]},
                {word_w[{proc_addr[4:2], 2'b01}]}, {word_w[{proc_addr[4:2], 2'b00}]}} = wdata_buf_r;
            end
    endcase
end
//==== sequential circuit =================================
always@( posedge clk or posedge proc_reset) begin
    if( proc_reset ) begin
        for (i=0;i<8;i=i+1) begin
            valid_r[i] <= 1'b0; // reset valid bit
            dirty_r[i] <= 1'b0; // reset dirty bit
            tag_r[i]   <= 25'b0; // reset tag
        end
        for (i=0;i<32;i=i+1) begin
            word_r[i]  <= 32'b0; // reset words
        end
        wdata_buf_r <= 0;
        mem_addr_buf_r <=0;
    end
    else begin
        for (i=0;i<8;i=i+1) begin
            valid_r[i] <= valid_w[i]; // reset valid bit
            dirty_r[i] <= dirty_w[i]; // reset dirty bit
            tag_r[i]   <= tag_w[i]; // reset tag
        end
        for (i=0;i<32;i=i+1) begin
            word_r[i]  <= word_w[i]; // reset words
        end
        wdata_buf_r <= wdata_buf_w;
        mem_addr_buf_r <= mem_addr_buf_w;
    end
end

endmodule