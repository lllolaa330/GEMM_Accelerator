import subprocess
import os

# 1. 待测试的网络层尺寸配置 (LLaMA3-8B)
layers_config = [
    {"layer": "q_proj",    "w_row": 4096,  "b_len": 4096},
    {"layer": "k_proj",    "w_row": 1024,  "b_len": 4096},
    {"layer": "v_proj",    "w_row": 1024,  "b_len": 4096},
    {"layer": "o_proj",    "w_row": 4096,  "b_len": 4096},
    {"layer": "gate_proj", "w_row": 14336, "b_len": 4096},
    {"layer": "up_proj",   "w_row": 14336, "b_len": 4096},
    {"layer": "down_proj", "w_row": 4096,  "b_len": 14336}
]

# 2. 待测试的 CU_TYPE 模式 (0:Static, 1:Dynamic, 2:Sliding, 3:DSP)
cu_types = [0, 1, 2, 3]

# 3. 指定所有需要编译的 Verilog 文件 
rtl_files = (
    "tb_GEMV_Accelerator_Top.sv " 
    "C:/Users/lllolaa/Desktop/GEMM_Accelerator/GEMM_Accelerator.srcs/sources_1/new/Accumulation_PE.sv "
    "C:/Users/lllolaa/Desktop/GEMM_Accelerator/GEMM_Accelerator.srcs/sources_1/new/Act_SI_Broadcast_Buffer.sv "
    "C:/Users/lllolaa/Desktop/GEMM_Accelerator/GEMM_Accelerator.srcs/sources_1/new/APE_Window.sv "
    "C:/Users/lllolaa/Desktop/GEMM_Accelerator/GEMM_Accelerator.srcs/sources_1/new/Array_Output_Accumulator.sv "
    "C:/Users/lllolaa/Desktop/GEMM_Accelerator/GEMM_Accelerator.srcs/sources_1/new/Compute_Core_Array.sv "
    "C:/Users/lllolaa/Desktop/GEMM_Accelerator/GEMM_Accelerator.srcs/sources_1/new/CU_wrapper.sv "
    "C:/Users/lllolaa/Desktop/GEMM_Accelerator/GEMM_Accelerator.srcs/sources_1/new/Data_DMA_Engine.sv "
    "C:/Users/lllolaa/Desktop/GEMM_Accelerator/GEMM_Accelerator.srcs/sources_1/new/dot_product_dsp.sv "
    "C:/Users/lllolaa/Desktop/GEMM_Accelerator/GEMM_Accelerator.srcs/sources_1/new/dsp_baseline_CU.sv "
    "C:/Users/lllolaa/Desktop/GEMM_Accelerator/GEMM_Accelerator.srcs/sources_1/new/Dynamic_Scoreboard_Core.sv "
    "C:/Users/lllolaa/Desktop/GEMM_Accelerator/GEMM_Accelerator.srcs/sources_1/new/Dynamic_Scoreboard_CU.sv "
    "C:/Users/lllolaa/Desktop/GEMM_Accelerator/GEMM_Accelerator.srcs/sources_1/new/GEMV_Accelerator_Top.sv "
    "C:/Users/lllolaa/Desktop/GEMM_Accelerator/GEMM_Accelerator.srcs/sources_1/new/Global_Controller.sv "
    "C:/Users/lllolaa/Desktop/GEMM_Accelerator/GEMM_Accelerator.srcs/sources_1/new/Hamming_Sorter_Dynamic.sv "
    "C:/Users/lllolaa/Desktop/GEMM_Accelerator/GEMM_Accelerator.srcs/sources_1/new/Hamming_Sorter_Static.sv "
    "C:/Users/lllolaa/Desktop/GEMM_Accelerator/GEMM_Accelerator.srcs/sources_1/new/Memory_Subsystem.sv "
    "C:/Users/lllolaa/Desktop/GEMM_Accelerator/GEMM_Accelerator.srcs/sources_1/new/PingPong_Weight_BRAM.sv "
    "C:/Users/lllolaa/Desktop/GEMM_Accelerator/GEMM_Accelerator.srcs/sources_1/new/Pipeline_Registers.sv "
    "C:/Users/lllolaa/Desktop/GEMM_Accelerator/GEMM_Accelerator.srcs/sources_1/new/Prefix_Buffer_LUTRAM.sv "
    "C:/Users/lllolaa/Desktop/GEMM_Accelerator/GEMM_Accelerator.srcs/sources_1/new/Prefix_Buffer_Window.sv "
    "C:/Users/lllolaa/Desktop/GEMM_Accelerator/GEMM_Accelerator.srcs/sources_1/new/Prefix_PE.sv "
    "C:/Users/lllolaa/Desktop/GEMM_Accelerator/GEMM_Accelerator.srcs/sources_1/new/Prefix_PE_Window.sv "
    "C:/Users/lllolaa/Desktop/GEMM_Accelerator/GEMM_Accelerator.srcs/sources_1/new/simple_fifo.sv "
    "C:/Users/lllolaa/Desktop/GEMM_Accelerator/GEMM_Accelerator.srcs/sources_1/new/SlidingWindow_CU.sv "
    "C:/Users/lllolaa/Desktop/GEMM_Accelerator/GEMM_Accelerator.srcs/sources_1/new/Sorter_and_SI.sv "
    "C:/Users/lllolaa/Desktop/GEMM_Accelerator/GEMM_Accelerator.srcs/sources_1/new/Static_Scoreboard_CU.sv "
    "C:/Users/lllolaa/Desktop/GEMM_Accelerator/GEMM_Accelerator.srcs/sources_1/new/URAM_Weight_Array.sv "
    "C:/Users/lllolaa/Desktop/GEMM_Accelerator/GEMM_Accelerator.srcs/sources_1/new/Window_Buffer.sv"
)

def run_cmd(cmd_list):
    """执行命令并检查是否报错"""
    # 将列表拼接成字符串打印，方便查看实际执行的命令
    print("Executing:", " ".join(cmd_list))
    # shell=True 在 Windows 下是必要的，用于识别系统环境变量路径
    result = subprocess.run(" ".join(cmd_list), shell=True, capture_output=True, text=True)
    if result.returncode != 0:
        print(f"\n[ERROR] Command Failed:\n{result.stderr}")
        return False
    return True

if __name__ == "__main__":
    # 初始化 CSV 表头（覆盖写入一次）
    csv_filename = "gemv_batch_results.csv"
    with open(csv_filename, "w") as f:
        f.write("CU_TYPE,Layer,W_ROW,B_LEN,GEMV_NUM,Total_Errors,Act_Ext2BRAM,Act_BRAM2CU,SI_Ext2BRAM,SI_BRAM2CU,Compute,Out_BRAM2Ext,Total_Cycles\n")

    # 开始遍历测试
    for cu in cu_types:
        for config in layers_config:
            layer = config["layer"]
            w_row = config["w_row"]
            b_len = config["b_len"]
            
            print(f"\n{'='*60}")
            print(f" STARTING SIMULATION: CU={cu} | Layer={layer} | W={w_row} | B={b_len}")
            print(f"{'='*60}")
            
            # (1) 编译步骤 xvlog (-sv 表示支持 SystemVerilog 语法，避免部分字符串处理报错)
            compile_cmd = ["xvlog", "-sv", rtl_files]
            if not run_cmd(compile_cmd): continue
            
            # (2) 顶层展开并动态传参 xelab
            # 注意 Windows 下传递 Verilog string 的转义要求非常严格：需要 \\" 
            xelab_cmd = [
                "xelab", 
                "-debug", "typical", 
                "-top", "tb_GEMV_Accelerator_Top", 
                "-snapshot", "batch_sim_snapshot",
                f'-generic_top "CU_TYPE={cu}"',
                f'-generic_top "target_layer=\\"{layer}\\""',  # 字符串参数的特殊转义
                f'-generic_top "WEIGHT_TOTAL_ROW={w_row}"',
                f'-generic_top "B_VEC_LEN={b_len}"'
            ]
            if not run_cmd(xelab_cmd): continue
            
            # (3) 运行仿真 xsim (-R 表示 run all，不打开图形界面直接在后台跑到底)
            xsim_cmd = ["xsim", "batch_sim_snapshot", "-R"]
            run_cmd(xsim_cmd)

    print(f"\n>>> Batch testing completely finished! Check {csv_filename} <<<")