#!/usr/bin/env python3
"""
MuJoCo 简单查看器

简单的 MuJoCo 查看器，用于可视化带执行器的模型。
"""

import mujoco
import mujoco.viewer
import numpy as np
import sys
from pathlib import Path

def main():
    """主函数"""
    # 获取脚本所在目录
    script_dir = Path(__file__).parent
    xml_file = script_dir / "model-actuator-position.xml"
    
    if not xml_file.exists():
        print(f"❌ 未找到XML文件: {xml_file}")
        print("💡 请确保在包含 model-actuator-position.xml 的目录中运行此脚本")
        return 1
    
    try:
        # 加载模型
        model = mujoco.MjModel.from_xml_path(str(xml_file))
        data = mujoco.MjData(model)
        
        print(f"✅ 模型加载成功")
        print(f"📊 执行器数量: {model.nu}")
        print(f"📊 关节数量: {model.njnt}")
        print(f"📊 自由度: {model.nv}")
        
        # 显示执行器信息
        print(f"📋 执行器列表:")
        for i in range(model.nu):
            name = mujoco.mj_id2name(model, mujoco.mjtObj.mjOBJ_ACTUATOR, i)
            joint_name = mujoco.mj_id2name(model, mujoco.mjtObj.mjOBJ_JOINT, i)
            print(f"   {i}: {name} -> {joint_name}")
        
        # 启动查看器
        print("\n🎮 启动 MuJoCo 查看器...")
        print("💡 使用鼠标控制视角，按 ESC 退出")
        
        with mujoco.viewer.launch(model, data) as viewer:
            while viewer.is_running():
                # 步进仿真
                mujoco.mj_step(model, data)
                
                # 同步查看器
                viewer.sync()
        
        print("\n👋 查看器已关闭")
        return 0
        
    except Exception as e:
        print(f"❌ 运行失败: {e}")
        import traceback
        traceback.print_exc()
        return 1

if __name__ == "__main__":
    sys.exit(main())
