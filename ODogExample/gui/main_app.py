"""
ODogExample 主应用入口 (兼容性包装器)

为了保持向后兼容性，原有的 main_app.py 现在作为新模块的包装器。
"""

try:
    from .app_entry import main
except ImportError:
    # 如果相对导入失败，尝试绝对导入
    import sys
    import os
    sys.path.append(os.path.dirname(os.path.dirname(os.path.abspath(__file__))))
    from gui.app_entry import main

# 为了保持向后兼容性，保留原有的类名和函数名
MainApplication = None  # 已移动到 app_main.py

def main():
    """主应用入口 - 重定向到新的应用入口"""
    from .app_entry import main as new_main
    return new_main()


if __name__ == "__main__":
    main()