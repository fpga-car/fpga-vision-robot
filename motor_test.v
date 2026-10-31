module motor_test(
    input        clk,        // 50MHz
    input        rx_esp,     // ESP32 TX -> FPGA
    input        rx_k210,    // K210  TX -> FPGA
    output       pwma,       // 左轮 PWM
    output reg   ain1,
    output reg   ain2,
    output       pwmb,       // 右轮 PWM
    output reg   bin1,
    output reg   bin2,
    output reg   led_debug
);

// ------------------------- 协议字符 -------------------------
localparam [7:0] C_HEAD = 8'h23;   // '#'
localparam [7:0] C_TAIL = 8'h3B;   // ';'
localparam [7:0] C_SEP  = 8'h2C;   // ','
localparam [7:0] C_COL  = 8'h3A;   // ':'  K210 新版用冒号分隔
localparam [7:0] C_NEG  = 8'h2D;   // '-'
localparam [7:0] C_DOT  = 8'h2E;   // '.' 小数部分直接丢弃（ESP32 常发 %.2f 格式）
localparam [7:0] C_ATT  = 8'h41;   // 'A' 姿态帧首字节
localparam [7:0] C_V    = 8'h56;   // 'V' 视觉帧首字节
localparam [7:0] C_VL   = 8'h76;   // 'v'
localparam [7:0] C_LF   = 8'h0A;   // '\n' 有的 K210 用换行结尾
localparam [7:0] C_CR   = 8'h0D;   // '\r'

localparam [7:0] CMD_F = 8'h46;    // 'F'
localparam [7:0] CMD_B = 8'h42;    // 'B'
localparam [7:0] CMD_L = 8'h4C;    // 'L'
localparam [7:0] CMD_R = 8'h52;    // 'R'
localparam [7:0] CMD_S = 8'h53;    // 'S'
localparam [7:0] CMD_A = 8'h41;    // 'A' 切自动
localparam [7:0] CMD_M = 8'h4D;    // 'M' 切手动
localparam [7:0] CMD_P = 8'h50;    // 'P' 兼容旧协议 #P:yaw,dist;

// ------------------------- 姿态参数 -------------------------
localparam integer TILT_LIMIT = 45;      // 度  车身倾斜保护阈值
localparam         TILT_EN    = 1'b1;    // 1: 自动模式下倾斜保护停车
localparam integer PITCH_HI   = 20;      // 度  坡度补偿高档
localparam integer PITCH_LO   = 10;      // 度  坡度补偿低档

// ------------------------- 避障参数 -------------------------
localparam         AVOID_WHEN_SEE = 1'b0; // 0: 识别到瓶子就不避障、一路走过去（追瓶模式）1: 看到瓶子也避障
localparam         AVOID_LEFT  = 1'b1;    // 1: 有障碍固定左转绕行（左轮慢右轮快）0: 固定右转绕行
localparam [8:0]  DIST_FAR  = 9'd60;     // cm  以下进入"谨慎"降速
localparam [8:0]  DIST_WARN = 9'd30;     // cm  以下弧线绕行
localparam [8:0]  DIST_NEAR = 9'd15;     // cm  以下倒车脱困
localparam [8:0]  DIST_HYST = 9'd40;     // cm  绕行解除（迟滞，防抖）
localparam [15:0] BRAKE_MS  = 16'd500;   // ms  减速时长（低速前进，逐渐停下）
localparam [15:0] STOP_MS   = 16'd300;   // ms  停稳等待（让车身静止再绕）
localparam [15:0] ARC_MIN_MS= 16'd800;   // ms  绕行最小时长（期间不再触发停车）


localparam integer VIS_IMG_W = 224;  // K210 画面宽度：QVGA+set_windowing((224,224)) -> 224；不做 windowing 的 QVGA -> 320
localparam integer VIS_LOST  = -1;
localparam [23:0] VIS_GAIN   = (200 * 1024) / VIS_IMG_W;      // Q10 定标，统一映射到 0~200
localparam [11:0] VIS_LOST_VAL = (VIS_LOST < 0) ? 12'hFFF : VIS_LOST[11:0];
localparam         VIS_ANY_NUM = 1'b1;   // 1: 允许"裸数字"起帧（K210 新版 x:y:w:h:conf\n 无前缀，必须开）
localparam         TRACK_STRAIGHT = 1'b1; // 1: 识别到瓶子就直线前行（忽略左右偏差） 0: 按偏差转向对准后再前进
localparam [15:0]  SEEN_HOLD  = 16'd3000; // ms  识别锁存：识别到一次后，丢失这么久内仍保持直行（抗检测闪烁）
localparam         ARRIVE_STOP = 1'b1;    // 1: 此刻看得见瓶子 且 靠近到 ARRIVE_DIST 内就停车（= 到达目标）
localparam [8:0]   ARRIVE_DIST = 9'd15;   // cm  判定"到达瓶子"的距离（嫌停太远改 10，会撞到改 20）
localparam         VIS_BOX_CENTER = 1'b1; // 1: 第0字段是框左上角，FPGA 自动算中心(x+w/2)；0: 第0字段已是中心

localparam [7:0]  POS_NONE = 8'd255;     // 内部：没看到瓶子
localparam [7:0]  POS_MID  = 8'd100;     // 画面中心（统一后的坐标 0~200）
localparam [7:0]  POS_DEAD = 8'd12;      // 中心死区 -> 直行
localparam [7:0]  POS_WIDE = 8'd35;      // 大偏差 -> 原地快转对准

// ------------------------- 超时 -------------------------
localparam [15:0] ATT_TMO  = 16'd600;    // ms  姿态帧超时
localparam [15:0] V_TMO    = 16'd500;    // ms  视觉帧超时
localparam [15:0] SWEEP_TMO= 16'd4000;   // ms  扫描换向超时（MPU 异常兜底）
localparam integer SWEEP_HALF = 45;      // 度  丢失目标单向扫描角 ±45°
localparam         GUARD_EN   = 1'b1;    // 1: 手动前进时的超声波保护（只拦 F，B/L/R 不拦）
localparam [8:0]  GUARD_DIST = 9'd15;    // cm  手动前进保护触发距离（误拦就把这里调小，如 8）

// ------------------------- 占空比（PWM 周期 4096 ≈ 12.2kHz）-------------------------
localparam [11:0] DUTY_0     = 12'd0;    //   0%
localparam [11:0] DUTY_BRAKE = 12'd614;  //  15%  避障减速（接近障碍时降速停车）
localparam [11:0] DUTY_AVD_F = 12'd1434; //  35%  避障外侧轮
localparam [11:0] DUTY_AVD_S = 12'd819;  //  20%  避障内侧轮
localparam [11:0] DUTY_CAU   = 12'd1843; //  45%  前方有物时的谨慎巡航
localparam [11:0] DUTY_ROT   = 12'd1434; //  35%  搜索旋转
localparam [11:0] DUTY_TURN  = 12'd1843; //  45%  大偏差原地转向
localparam [11:0] DUTY_ARC_F = 12'd1843; //  45%  弧线修正外侧轮
localparam [11:0] DUTY_ARC_S = 12'd1024; //  25%  弧线修正内侧轮
localparam [11:0] DUTY_RUN   = 12'd2458; //  60%  正常巡航
localparam [11:0] DUTY_MAN   = 12'd3072; //  75%  手动直行
localparam [11:0] COMP_HI    = 12'd307;  //  7.5% 坡度补偿（坡度大）
localparam [11:0] COMP_LO    = 12'd164;  //  4.0% 坡度补偿（坡度小）

// ------------------------- 两路串口 -------------------------
wire [7:0] rx_data_esp;   wire rx_done_esp;
wire [7:0] rx_data_k210;  wire rx_done_k210;

uart_rx_esp #(.CLK_FREQ(50000000), .BAUD(9600)) u_uart_esp(
    .clk(clk), .rst_n(1'b1), .rx(rx_esp),
    .data_out(rx_data_esp), .rx_done(rx_done_esp)
);
uart_rx_k210 #(.CLK_FREQ(50000000), .BAUD(115200)) u_uart_k210(
    .clk(clk), .rst_n(1'b1), .rx(rx_k210),
    .data_out(rx_data_k210), .rx_done(rx_done_k210)
);

// ------------------------- 毫秒时基 -------------------------
reg [15:0] ms_div = 16'd0;
reg        ms_tick = 1'b0;
reg [15:0] ms_cnt = 16'd0;

always @(posedge clk) begin
    if(ms_div >= 16'd49999) begin
        ms_div  <= 16'd0;
        ms_tick <= 1'b1;
    end
    else begin
        ms_div  <= ms_div + 1'b1;
        ms_tick <= 1'b0;
    end
end

always @(posedge clk) begin
    if(ms_tick) ms_cnt <= ms_cnt + 1'b1;
    if(rx_done_k210) v_burst <= 16'd0;
    else if(ms_tick && (v_burst < 16'd1000)) v_burst <= v_burst + 1'b1;
end

// ------------------------- 状态寄存器 -------------------------
reg        auto_mode = 1'b0;          // 1:自动追踪  0:手动
reg [7:0]  cmd_reg   = CMD_S;         // 手动指令

// ---- ESP32 帧解析 ----
localparam [2:0] S_IDLE  = 3'd0;      // 等帧首字节 'A' 或 '#'
localparam [2:0] S_TYPE  = 3'd1;      // '#' 之后：命令字符
localparam [2:0] S_CMD   = 3'd2;      // 命令帧体，等 ';'
localparam [2:0] S_ATT   = 3'd3;      // 姿态帧体
localparam [2:0] S_ATYPE = 3'd4;      // '#'+'A' 之后：';' = 切自动指令，':'/数字 = 姿态帧
reg [2:0]  esp_st  = S_IDLE;
reg [7:0]  esp_type = CMD_S;
reg [1:0]  a_field  = 2'd0;           // 0:roll 1:pitch 2:yaw 3:dist
reg [15:0] a_tmp    = 16'd0;          // 当前字段数值（不含符号）
reg        a_neg    = 1'b0;           // 当前字段为负
reg        a_frac   = 1'b0;           // 已到小数点之后：小数位丢弃（"25.0" -> 25）

// ---- 姿态角 ----
reg signed [9:0] roll_reg  = 10'd0;   // -180 ~ 180
reg signed [9:0] pitch_reg = 10'd0;   // -180 ~ 180
reg        [8:0] yaw_reg   = 9'd0;    // 0 ~ 359
reg        [8:0] dist_reg  = 9'd400;  // cm，上电默认无障碍
reg        [15:0] t_att    = 16'd0;   // 最近一次姿态帧时刻


reg        k_on     = 1'b0;           // 帧接收中
reg        k_multi  = 1'b0;           // 1: #x,y,w,h,conf; 多字段   0: 单值 V:cx;
reg [2:0]  k_field  = 3'd0;           // 当前字段号 0:x 1:y 2:w 3:h 4:conf
reg        k_frac   = 1'b0;
reg [15:0] k_x = 16'd0, k_y = 16'd0;  // 框中心坐标
reg [15:0] k_w = 16'd0, k_h = 16'd0;  // 框尺寸（=0 即没识别到）
reg [11:0] v_tmp    = 12'd0;          // 单值模式累加
reg        v_neg    = 1'b0;           // 单值模式负号（-1 表示没看到）
reg [15:0] v_burst  = 16'd0;          // 距上一个字节的间隔(ms)：区分同一帧的多个目标
reg [31:0] v_best   = 32'd0;          // 当前突发中最大框面积
reg [7:0]  cx_reg   = POS_NONE;
reg [15:0] t_v      = 16'd0;

// ---- 丢失目标扫描 ----
reg        sweep_dir = 1'b0;          // 0:右转  1:左转
reg [8:0]  sweep_org = 9'd0;          // 扫描原点：丢失目标瞬间的车头方向
reg [15:0] sweep_ms  = 16'd0;
reg signed [9:0] dyaw = 10'd0;        // 相对原点的角度差

// ---- 避障 FSM ----
localparam [1:0] AV_FREE  = 2'd0;     // 无障碍
localparam [1:0] AV_ARC   = 2'd1;     // 左转弧线绕行
localparam [1:0] AV_BRAKE = 2'd2;     // 减速 -> 停稳（准备绕行）
reg [1:0]  av_state = AV_FREE;
reg [15:0] av_ms    = 16'd0;
reg        obs_flag = 1'b0;           // <30cm（迟滞，>40 解除）
reg        obs_near = 1'b0;           // <15cm（>30 解除）

wire att_timeout = ((ms_cnt - t_att) >= ATT_TMO);
wire v_timeout   = ((ms_cnt - t_v)   >= V_TMO);
wire att_valid   = !att_timeout;                      // 姿态数据有效
wire bottle_lost = (cx_reg == POS_NONE) || v_timeout; // 这一刻看没看到水瓶


reg        seen_latch = 1'b0;
reg [15:0] t_seen     = 16'd0;            // 最近一次看到瓶子的时刻
always @(posedge clk) begin
    if(!auto_mode)                      seen_latch <= 1'b0;              // 手动模式清掉
    else if(!bottle_lost) begin         seen_latch <= 1'b1; t_seen <= ms_cnt; end
    else if(seen_latch && ((ms_cnt - t_seen) >= SEEN_HOLD))
                                        seen_latch <= 1'b0;              // 确认真丢了
end
wire bottle_gone = !seen_latch;                                          // 真丢了才去搜索

wire tilt_alarm = TILT_EN && att_valid &&
                  ((roll_reg >  TILT_LIMIT || roll_reg < -TILT_LIMIT) ||
                   (pitch_reg > TILT_LIMIT || pitch_reg < -TILT_LIMIT));

reg signed [10:0] att_v = 11'd0;
always @(*) begin
    att_v = a_neg ? -$signed({1'b0, a_tmp}) : $signed({1'b0, a_tmp});
end

reg [23:0] v_prod = 24'd0;
always @(*) v_prod = {12'd0, v_tmp} * VIS_GAIN;
wire [13:0] v_pos  = v_prod[23:10];                       // 单值模式 >>10

wire [15:0] k_cx = VIS_BOX_CENTER ? (k_x + {1'b0, k_w[15:1]}) : k_x;

reg [23:0] k_prod = 24'd0;
always @(*) k_prod = {8'd0, k_cx} * VIS_GAIN;
wire [13:0] k_pos  = k_prod[23:10];                        // 多字段模式 >>10
wire [31:0] k_area = {16'd0, k_w} * {16'd0, k_h};            // 框面积，用于多目标择优

localparam         DIST_FILT_EN = 1'b1;   // 1: 开启距离中值滤波  0: 直接用原始值
reg [8:0] d_s1 = 9'd400, d_s2 = 9'd400;   // 前两次采样

function [8:0] med3;
    input [8:0] a, b, c;
    begin
        if(a <= b) begin
            if(b <= c)      med3 = b;
            else if(a <= c) med3 = c;
            else            med3 = a;
        end
        else begin
            if(a <= c)      med3 = a;
            else if(b <= c) med3 = c;
            else            med3 = b;
        end
    end
endfunction

wire [8:0] d_new = (a_tmp > 16'd400) ? 9'd400 : (a_neg ? 9'd0 : a_tmp[8:0]);

wire       v_is_end  = (rx_data_k210 == C_TAIL) ||
                      (rx_data_k210 == C_LF)   ||
                      (rx_data_k210 == C_CR);
wire       k_is_lost = (k_w == 16'd0) || (k_h == 16'd0) ||
                      (k_x == 16'd0 && k_y == 16'd0);

reg signed [9:0] dyaw_raw = 10'd0;
always @(*) begin
    dyaw_raw = $signed({1'b0, yaw_reg}) - $signed({1'b0, sweep_org});
    if(dyaw_raw >  180) dyaw = dyaw_raw - 360;        // 处理 0/360 环绕
    else if(dyaw_raw < -180) dyaw = dyaw_raw + 360;
    else dyaw = dyaw_raw;
end

always @(posedge clk) begin

    // ---------------- ESP32 帧解析 ----------------
    if(rx_done_esp) begin
        case(esp_st)
        S_IDLE: begin
            if(rx_data_esp == C_ATT) begin            // 姿态帧：A:...;
                a_field  <= 2'd0;
                a_tmp    <= 16'd0;
                a_neg    <= 1'b0;
                a_frac   <= 1'b0;
                esp_st   <= S_ATT;
            end
            else if(rx_data_esp == C_HEAD) begin       // 指令帧：#X;
                esp_st   <= S_TYPE;
            end
        end
        S_TYPE: begin
            if(rx_data_esp == C_ATT) begin          // '#A' 待定：可能是 #A;(切自动)，也可能是 #A:...;(姿态帧)
                esp_st <= S_ATYPE;
            end
            else if(rx_data_esp == CMD_P) begin      // 兼容旧协议 #P:yaw,dist;
                esp_st  <= S_ATT;
                a_field <= 2'd2;                    // 第一个字段就是 yaw
                a_tmp   <= 16'd0;
                a_neg   <= 1'b0;
                a_frac  <= 1'b0;
            end
            else begin                               // F/B/L/R/S/M
                esp_type <= rx_data_esp;
                esp_st   <= S_CMD;
            end
        end
        S_ATYPE: begin
            if(rx_data_esp == C_TAIL) begin          // "#A;" -> 切自动指令
                esp_st    <= S_IDLE;
                auto_mode <= 1'b1;
                sweep_org <= yaw_reg;
                sweep_dir <= 1'b0;
                sweep_ms  <= 16'd0;
                av_state  <= AV_FREE;
                av_ms     <= 16'd0;
            end
            else if(rx_data_esp == C_HEAD) begin     // 新指令帧
                esp_st <= S_TYPE;
            end
            else begin                               // '#'+'A' 后面还有数据 -> 按姿态帧解析
                esp_st  <= S_ATT;
                a_field <= 2'd0;
                a_tmp   <= 16'd0;
                a_neg   <= 1'b0;
                a_frac  <= 1'b0;
                if(rx_data_esp == C_NEG) a_neg <= 1'b1;
                else if(rx_data_esp >= 8'h30 && rx_data_esp <= 8'h39)
                    a_tmp <= {12'd0, rx_data_esp[3:0]};   // 本字节就是第一个数字
            end
        end
        S_CMD: begin
            if(rx_data_esp == C_TAIL) begin            // 指令帧结束
                esp_st <= S_IDLE;
                if(esp_type == CMD_A) begin            // 切自动：以当前车头为扫描原点
                    auto_mode <= 1'b1;
                    sweep_org <= yaw_reg;
                    sweep_dir <= 1'b0;
                    sweep_ms  <= 16'd0;
                    av_state  <= AV_FREE;
                    av_ms     <= 16'd0;
                end
                else if(esp_type == CMD_M) begin       // 切手动：先停车等指令
                    auto_mode <= 1'b0;
                    cmd_reg   <= CMD_S;
                end
                else begin                             // F/B/L/R/S 直接进入手动
                    auto_mode <= 1'b0;
                    cmd_reg   <= esp_type;
                end
            end
            else if(rx_data_esp == C_HEAD) begin       // 异常恢复
                esp_st <= S_TYPE;
            end
        end
        S_ATT: begin
            if(rx_data_esp == C_TAIL) begin            // 姿态帧结束：最后一个字段是 dist
                esp_st   <= S_IDLE;
                t_att    <= ms_cnt;
                if(a_field == 2'd0) roll_reg <= att_v[9:0];
                else if(a_field == 2'd1) pitch_reg <= att_v[9:0];
                else if(a_field == 2'd2) begin
                    if(att_v < 0)         yaw_reg <= att_v[8:0] + 9'd360;
                    else if(att_v >= 360) yaw_reg <= att_v[8:0] - 9'd360;
                    else                  yaw_reg <= att_v[8:0];
                end
                else begin
                    d_s2 <= d_s1;
                    d_s1 <= d_new;
                    dist_reg <= DIST_FILT_EN ? med3(d_new, d_s1, d_s2) : d_new;
                end
            end
            else if(rx_data_esp == C_ATT) begin        // 上一帧没结尾就来了新帧，重新同步
                a_field <= 2'd0;
                a_tmp   <= 16'd0;
                a_neg   <= 1'b0;
                a_frac  <= 1'b0;
            end
            else if(rx_data_esp == C_HEAD) begin       // 姿态帧中途插入指令帧
                esp_st   <= S_TYPE;
            end
            else if(rx_data_esp == C_SEP) begin        // ',' 分隔字段
                if(a_field == 2'd0)      roll_reg  <= att_v[9:0];
                else if(a_field == 2'd1) pitch_reg <= att_v[9:0];
                else if(a_field == 2'd2) begin
                    if(att_v < 0)         yaw_reg <= att_v[8:0] + 9'd360;
                    else if(att_v >= 360) yaw_reg <= att_v[8:0] - 9'd360;
                    else                  yaw_reg <= att_v[8:0];
                end
                a_field <= a_field + 1'b1;
                a_tmp   <= 16'd0;
                a_neg   <= 1'b0;
                a_frac  <= 1'b0;
            end
            else if(rx_data_esp == C_NEG) begin        // 负号
                a_neg <= 1'b1;
            end
            else if(rx_data_esp == C_DOT) begin        // 小数点：后面几位是小数，丢弃
                a_frac <= 1'b1;
            end
            else if(rx_data_esp >= 8'h30 && rx_data_esp <= 8'h39) begin
                if(!a_frac) a_tmp <= a_tmp * 10 + {8'd0, rx_data_esp[3:0]};  // ':' 等非数字忽略
            end
        end
        endcase
    end

    if(rx_done_k210) begin
        t_v <= ms_cnt;
        if(!k_on) begin
            if(rx_data_k210 == C_V || rx_data_k210 == C_VL) begin       // 单值帧
                k_on <= 1'b1; k_multi <= 1'b0;
                v_tmp <= 12'd0; v_neg <= 1'b0; k_frac <= 1'b0;
            end
            else if(rx_data_k210 == C_HEAD) begin                     // '#' 多字段帧
                k_on <= 1'b1; k_multi <= 1'b1;
                k_field <= 3'd0; v_tmp <= 12'd0; v_neg <= 1'b0; k_frac <= 1'b0;
            end
            else if(VIS_ANY_NUM && rx_data_k210 >= 8'h30 && rx_data_k210 <= 8'h39) begin
                k_on <= 1'b1; k_multi <= 1'b1;                          // 裸数字开头
                k_field <= 3'd0; v_tmp <= {8'd0, rx_data_k210[3:0]};
                v_neg <= 1'b0; k_frac <= 1'b0;
            end
        end
        else begin
            if(v_is_end) begin                                      // 帧尾 ';' / '\n' / '\r'
                k_on <= 1'b0;
                if(k_multi && (k_field >= 3'd4)) begin             // 5 字段齐了
                    if(k_is_lost) begin
                        v_best <= 32'd0;
                        cx_reg <= POS_NONE;                          // 没识别到
                    end
                    else if((v_burst >= 16'd20) || (k_area > v_best)) begin
                        v_best <= k_area;
                        cx_reg <= (k_pos > 14'd254) ? 8'd254 : k_pos[7:0];
                    end
                end
                else if(v_neg || (v_tmp == VIS_LOST_VAL) || (v_tmp > 12'd254)) begin
                    cx_reg <= POS_NONE;                              // 单值：-1 / 255 = 没看到
                end
                else begin
                    cx_reg <= v_pos[7:0];
                end
            end
            else if(rx_data_k210 == C_HEAD) begin                      // 中途来了新帧：重新同步
                k_on <= 1'b1; k_multi <= 1'b1;
                k_field <= 3'd0; v_tmp <= 12'd0; v_neg <= 1'b0; k_frac <= 1'b0;
            end
            else if(rx_data_k210 == C_V || rx_data_k210 == C_VL) begin
                k_on <= 1'b1; k_multi <= 1'b0;
                v_tmp <= 12'd0; v_neg <= 1'b0; k_frac <= 1'b0;
            end
            else if(rx_data_k210 == C_SEP || rx_data_k210 == C_COL) begin  // ',' / ':' 字段分隔
                if(k_multi) begin
                    case(k_field)
                    3'd0: k_x <= v_tmp;
                    3'd1: k_y <= v_tmp;
                    3'd2: k_w <= v_tmp;
                    3'd3: k_h <= v_tmp;
                    default: ;                                       // conf 丢弃
                    endcase
                    if(k_field < 3'd4) k_field <= k_field + 1'b1;
                    v_tmp  <= 12'd0;
                    v_neg  <= 1'b0;
                    k_frac <= 1'b0;
                end
            end
            else if(rx_data_k210 == C_NEG) begin                     // 负号
                if(!k_multi) v_neg <= 1'b1;
            end
            else if(rx_data_k210 == C_DOT) begin                     // 小数点：小数位丢弃
                k_frac <= 1'b1;
            end
            else if(rx_data_k210 >= 8'h30 && rx_data_k210 <= 8'h39) begin
                if(!k_frac) v_tmp <= v_tmp * 10 + {8'd0, rx_data_k210[3:0]};
            end
        end
    end


    if(!obs_flag) begin
        if(dist_reg < DIST_WARN) obs_flag <= 1'b1;
    end
    else if(dist_reg > DIST_HYST) begin
        obs_flag <= 1'b0;
    end

    if(!obs_near) begin
        if(dist_reg < DIST_NEAR) obs_near <= 1'b1;
    end
    else if(dist_reg > DIST_WARN) begin
        obs_near <= 1'b0;
    end


    case(av_state)
    AV_FREE: begin
        av_ms <= 16'd0;
        if(obs_near)      av_state <= AV_BRAKE;   // <15cm：先减速停住，再绕
        else if(obs_flag) av_state <= AV_ARC;     // 15~30cm：直接左转绕行
    end
    AV_BRAKE: begin

        if(av_ms < (BRAKE_MS + STOP_MS)) av_ms <= av_ms + {15'd0, ms_tick};
        else begin
            av_state <= AV_ARC;
            av_ms    <= 16'd0;
        end
    end
    AV_ARC: begin
        if(av_ms < ARC_MIN_MS) av_ms <= av_ms + {15'd0, ms_tick};
        if(!obs_flag) begin                                   // 绕过去了（>40cm）
            av_state <= AV_FREE;
        end
        else if(obs_near && (av_ms >= ARC_MIN_MS)) begin        // 绕行途中又贴太近：再停一次再绕
            av_state <= AV_BRAKE;
            av_ms    <= 16'd0;
        end
    end
    default: begin
        av_state <= AV_FREE;
        av_ms    <= 16'd0;
    end
    endcase

    if(!auto_mode || !bottle_gone) begin
        sweep_org <= yaw_reg;          // 锁存有效时，扫描原点跟随车头方向
        sweep_ms  <= 16'd0;
    end
    else begin
        if(sweep_ms < SWEEP_TMO) sweep_ms <= sweep_ms + {15'd0, ms_tick};

        if( (att_valid && (dyaw >= SWEEP_HALF || dyaw <= -SWEEP_HALF)) ||
            (sweep_ms >= SWEEP_TMO) ) begin
            sweep_dir <= ~sweep_dir;
            sweep_ms  <= 16'd0;
        end
    end
end

localparam [2:0] ACT_STOP   = 3'd0;
localparam [2:0] ACT_SEARCH = 3'd1;
localparam [2:0] ACT_BRAKE  = 3'd2;
localparam [2:0] ACT_ARC    = 3'd3;
localparam [2:0] ACT_TRACK  = 3'd4;

wire avoid_en = (AVOID_WHEN_SEE || bottle_gone);

reg [2:0] act_sel = ACT_STOP;
always @(*) begin
    if(tilt_alarm)                              act_sel = ACT_STOP;   // 倾斜保护
    else if(avoid_en && (av_state == AV_BRAKE)) act_sel = ACT_BRAKE;  // 避障：<15cm 减速停住
    else if(avoid_en && (av_state == AV_ARC))   act_sel = ACT_ARC;    // 避障：15~30cm 左转绕行

    else if(ARRIVE_STOP && !bottle_lost &&
            (dist_reg <= ARRIVE_DIST))          act_sel = ACT_STOP;   // 已到达瓶子：停车
    else if(bottle_gone)                        act_sel = ACT_SEARCH; // 确认丢了才搜索
    else                                        act_sel = ACT_TRACK;  // 识别到过：保持直行
end

reg        whl_en   = 1'b0;
reg        dir_a    = 1'b1;           // 1:前进 0:后退
reg        dir_b    = 1'b1;
reg [11:0] duty_ba  = 12'd0;          // 基础占空比（未加坡度补偿）
reg [11:0] duty_bb  = 12'd0;
reg        slope_en = 1'b0;           // 是否施加坡度补偿

always @(*) begin
    if(auto_mode) begin
        case(act_sel)
        ACT_STOP: begin

            whl_en = 1'b0; dir_a = 1'b1; dir_b = 1'b1;
            duty_ba = DUTY_0; duty_bb = DUTY_0; slope_en = 1'b0;
        end
        ACT_BRAKE: begin

            if(av_ms < BRAKE_MS) begin
                whl_en = 1'b1; dir_a = 1'b1; dir_b = 1'b1;      // 减速中
                duty_ba = DUTY_BRAKE; duty_bb = DUTY_BRAKE; slope_en = 1'b0;
            end
            else begin
                whl_en = 1'b0; dir_a = 1'b1; dir_b = 1'b1;      // 停稳等待
                duty_ba = DUTY_0; duty_bb = DUTY_0; slope_en = 1'b0;
            end
        end
        ACT_ARC: begin

            whl_en = 1'b1; dir_a = 1'b1; dir_b = 1'b1;
            if(AVOID_LEFT) begin duty_ba = DUTY_AVD_S; duty_bb = DUTY_AVD_F; end  // 左绕
            else           begin duty_ba = DUTY_AVD_F; duty_bb = DUTY_AVD_S; end  // 右绕
            slope_en = 1'b1;
        end
        ACT_SEARCH: begin

            whl_en = 1'b1;
            dir_a  = !sweep_dir;
            dir_b  =  sweep_dir;
            duty_ba = DUTY_ROT; duty_bb = DUTY_ROT; slope_en = 1'b0;
        end
        default: begin                  // ACT_TRACK：视觉追踪
            if(TRACK_STRAIGHT) begin

                whl_en = 1'b1; dir_a = 1'b1; dir_b = 1'b1;
                if(dist_reg < DIST_FAR) begin duty_ba = DUTY_CAU; duty_bb = DUTY_CAU; end
                else                     begin duty_ba = DUTY_RUN; duty_bb = DUTY_RUN; end
                slope_en = 1'b1;
            end
            else if(cx_reg > (POS_MID + POS_WIDE) || cx_reg < (POS_MID - POS_WIDE)) begin

                whl_en = 1'b1;
                dir_a  = (cx_reg > POS_MID);      // 瓶子在右 -> 右转 -> 左轮前进
                dir_b  = (cx_reg < POS_MID);      // 瓶子在左 -> 左转 -> 右轮前进
                duty_ba = DUTY_TURN; duty_bb = DUTY_TURN; slope_en = 1'b0;
            end
            else if(cx_reg > (POS_MID + POS_DEAD) || cx_reg < (POS_MID - POS_DEAD)) begin

                whl_en = 1'b1; dir_a = 1'b1; dir_b = 1'b1;
                if(cx_reg > POS_MID) begin duty_ba = DUTY_ARC_F; duty_bb = DUTY_ARC_S; end
                else                 begin duty_ba = DUTY_ARC_S; duty_bb = DUTY_ARC_F; end
                slope_en = 1'b1;
            end
            else begin

                whl_en = 1'b1; dir_a = 1'b1; dir_b = 1'b1;
                if(dist_reg < DIST_FAR) begin duty_ba = DUTY_CAU; duty_bb = DUTY_CAU; end
                else                     begin duty_ba = DUTY_RUN; duty_bb = DUTY_RUN; end
                slope_en = 1'b1;
            end
        end
        endcase
    end
    else begin

        case(cmd_reg)
        CMD_F: begin
            if(GUARD_EN && (dist_reg < GUARD_DIST)) begin  // 手动前进防撞
                whl_en = 1'b0; dir_a = 1'b1; dir_b = 1'b1;
                duty_ba = DUTY_0; duty_bb = DUTY_0; slope_en = 1'b0;
            end
            else begin
                whl_en = 1'b1; dir_a = 1'b1; dir_b = 1'b1;
                duty_ba = DUTY_MAN; duty_bb = DUTY_MAN; slope_en = 1'b1;
            end
        end
        CMD_B: begin
            whl_en = 1'b1; dir_a = 1'b0; dir_b = 1'b0;
            duty_ba = DUTY_MAN; duty_bb = DUTY_MAN; slope_en = 1'b1;
        end
        CMD_L: begin                                        // 左转：A后 B前
            whl_en = 1'b1; dir_a = 1'b0; dir_b = 1'b1;
            duty_ba = DUTY_TURN; duty_bb = DUTY_TURN; slope_en = 1'b0;
        end
        CMD_R: begin                                        // 右转：A前 B后
            whl_en = 1'b1; dir_a = 1'b1; dir_b = 1'b0;
            duty_ba = DUTY_TURN; duty_bb = DUTY_TURN; slope_en = 1'b0;
        end
        default: begin                                      // 停止
            whl_en = 1'b0; dir_a = 1'b1; dir_b = 1'b1;
            duty_ba = DUTY_0; duty_bb = DUTY_0; slope_en = 1'b0;
        end
        endcase
    end
end

reg signed [11:0] comp = 12'd0;
always @(*) begin
    if(!slope_en)                   comp = 12'd0;
    else if(pitch_reg >=  PITCH_HI) comp =  COMP_HI;    // 上坡：加力
    else if(pitch_reg >=  PITCH_LO) comp =  COMP_LO;
    else if(pitch_reg <= -PITCH_HI) comp = -COMP_HI;    // 下坡：减力
    else if(pitch_reg <= -PITCH_LO) comp = -COMP_LO;
    else                            comp = 12'd0;
end

reg signed [12:0] duty_sa = 13'd0;
reg signed [12:0] duty_sb = 13'd0;
always @(*) begin
    duty_sa = $signed({1'b0, duty_ba}) + comp;
    if(duty_sa < 0)     duty_sa = 0;
    if(duty_sa > 4095)  duty_sa = 4095;

    duty_sb = $signed({1'b0, duty_bb}) + comp;
    if(duty_sb < 0)     duty_sb = 0;
    if(duty_sb > 4095)  duty_sb = 4095;
end

always @(posedge clk) begin
    ain1 <= whl_en &  dir_a;
    ain2 <= whl_en & ~dir_a;
    bin1 <= whl_en &  dir_b;
    bin2 <= whl_en & ~dir_b;
end

reg [11:0] pwm_cnt = 12'd0;
always @(posedge clk) pwm_cnt <= pwm_cnt + 1'b1;

assign pwma = (pwm_cnt < duty_sa[11:0]);
assign pwmb = (pwm_cnt < duty_sb[11:0]);

always @(posedge clk) begin
    if(tilt_alarm)              led_debug <= ms_cnt[6];                 // 8Hz  倾斜报警
    else if(obs_flag)           led_debug <= ms_cnt[5];                 // 16Hz 超声波 <30cm
    else if(!auto_mode)         led_debug <= ms_cnt[9];                 // 1Hz  手动
    else if(v_timeout)          led_debug <= ms_cnt[7];                 // 4Hz  收不到 K210 数据
    else if(bottle_gone)        led_debug <= !ms_cnt[8] & ms_cnt[6];    // 双闪 确认丢失->搜索
    else                        led_debug <= 1'b1;                      // 常亮 锁定/保持直行
end

endmodule
