"""
MaojocoConverter - Fusion 360 to MuJoCo 数据转换器

主转换器类，协调整个转换流程
"""

from pathlib import Path
from typing import Optional
from datetime import datetime

from .utils.logger import logger
from .type_definitions import ConversionConfig
from .config import create_config_from_dict
from .context import MaojocoContext
from .phases import PhaseManager


class ConversionResult:
    """转换结果"""
    
    def __init__(self, success: bool, message: str = "", data: Optional[dict] = None):
        self.success = success
        self.message = message
        self.data = data or {}
        self.timestamp = datetime.now()


class MaojocoConverter:
    """主转换器 - 协调整个转换流程"""
    
    def __init__(self, config: Optional[dict] = None):
        """
        初始化转换器
        
        Args:
            config: 转换配置字典或ConversionConfig对象
        """
        if isinstance(config, ConversionConfig):
            self.config = config
            self.config_dict = self.config.to_dict()
        else:
            self.config_dict = config or {}
            self.config = create_config_from_dict(self.config_dict)
        self.ctx = None  # 将在初始化时创建 MaojocoContext
        self.phase_manager = None  # 将在初始化时创建 PhaseManager
        
    def convert(self, export_dir: str) -> ConversionResult:
        """
        执行转换流程
        
        Args:
            export_dir: 导出数据目录路径
            
        Returns:
            ConversionResult: 转换结果
        """
        try:
            logger.info(f"🚀 开始转换流程，导出目录: {export_dir}")
            
            # 验证导出目录
            export_path = Path(export_dir)
            if not export_path.exists():
                return ConversionResult(False, f"导出目录不存在: {export_dir}")
            
            # 阶段1: 初始化
            logger.info("📋 阶段1: 初始化转换器")
            self._initialize_converter(export_path, self.config_dict.get('with_actuators', False))
            
            # 阶段2: 数据加载（将在第4步实现）
            logger.info("📋 阶段2: 数据加载")
            self._load_data()
            
            # 阶段3: 关系分析（将在第6步实现）
            logger.info("📋 阶段3: 关系分析")
            self._analyze_relationships()
            
            # 阶段4: 单位转换（将在第8步实现）
            logger.info("📋 阶段4: 单位转换")
            self._convert_units()
            
            # 阶段5: 模型生成（将在第9步实现）
            logger.info("📋 阶段5: 模型生成")
            self._generate_model()
            
            # 阶段6: 执行器生成（可选）
            if self.ctx._generate_actuators:
                logger.info("📋 阶段6: 执行器生成")
                self._generate_actuators()
                actuator_phase = 6
                total_phases = 7
            else:
                logger.info("📋 跳过执行器生成阶段")
                actuator_phase = 0
                total_phases = 6
            
            # 最终阶段: 完成转换
            logger.info(f"📋 阶段{actuator_phase + 1 if actuator_phase > 0 else 6}: 完成转换")
            result = self._complete_conversion(total_phases)
            
            logger.success("转换流程完成")
            return result
            
        except Exception as e:
            logger.error_msg(f"转换失败: {e}")
            return ConversionResult(False, str(e))
    
    def _initialize_converter(self, export_path: Path, with_actuators: bool = False):
        """初始化转换器"""
        logger.info(f"📁 导出目录: {export_path}")
        logger.info(f"⚙️  配置: {self.config_dict}")
        
        # 创建 MaojocoContext
        self.ctx = MaojocoContext(str(export_path), self.config)
        
        # 设置执行器生成标志
        self.ctx._generate_actuators = with_actuators
        
        # 创建 PhaseManager
        self.phase_manager = PhaseManager(self.ctx)
        
                
        # 检查必要的文件
        required_files = ['component_positions.json', 'export_description.md']
        for file_name in required_files:
            file_path = export_path / file_name
            if not file_path.exists():
                raise FileNotFoundError(f"必要的文件不存在: {file_name}")
            logger.info(f"✅ 找到文件: {file_name}")
        
        logger.success("初始化完成")
    
    def _load_data(self):
        """数据加载阶段"""
        logger.info("🔄 数据加载阶段")
        if self.phase_manager:
            success = self.phase_manager.execute_phase('data_loading')
            if not success:
                raise RuntimeError("数据加载阶段失败")
        else:
            logger.warning("⚠️  PhaseManager 未初始化")
    
    def _analyze_relationships(self):
        """关系分析阶段"""
        logger.info("🔄 关系分析阶段")
        if self.phase_manager:
            success = self.phase_manager.execute_phase('relationship_analysis')
            if not success:
                raise RuntimeError("关系分析阶段失败")
        else:
            logger.warning("⚠️  PhaseManager 未初始化")
    
    def _convert_units(self):
        """单位转换阶段"""
        logger.info("🔄 单位转换阶段")
        if self.phase_manager:
            success = self.phase_manager.execute_phase('unit_conversion')
            if not success:
                raise RuntimeError("单位转换阶段失败")
        else:
            logger.warning("⚠️  PhaseManager 未初始化")
    
    def _generate_model(self):
        """模型生成阶段"""
        logger.info("🔄 模型生成阶段")
        if self.phase_manager:
            success = self.phase_manager.execute_phase('model_generation')
            if not success:
                raise RuntimeError("模型生成阶段失败")
        else:
            logger.warning("⚠️  PhaseManager 未初始化")
    
    def _generate_actuators(self):
        """执行器生成阶段"""
        logger.info("⚡ 执行器生成阶段")
        if self.phase_manager:
            success = self.phase_manager.execute_phase('actuator_generation')
            if not success:
                raise RuntimeError("执行器生成阶段失败")
        else:
            logger.warning("⚠️  PhaseManager 未初始化")
    
    def _complete_conversion(self, total_phases: int = 6) -> ConversionResult:
        """完成转换"""
        logger.info("📊 转换统计:")
        logger.info(f"    - 处理阶段: {total_phases}/{total_phases}")
        logger.info("    - 状态: 成功")
        
        return ConversionResult(
            success=True,
            message="转换成功完成",
            data={
                'phases_completed': total_phases,
                'export_directory': str(self.ctx.export_dir) if self.ctx else None,
                'timestamp': datetime.now().isoformat()
            }
        )
    
    def get_status(self) -> dict:
        """获取转换器状态"""
        return {
            'initialized': self.ctx is not None,
            'current_phase': self.ctx.phase if self.ctx else None,
            'config': self.config.to_dict() if self.config else {}
        }


def main():
    """命令行入口"""
    import argparse
    import json
    
    parser = argparse.ArgumentParser(description='MaojocoConverter - Fusion 360 to MuJoCo 转换器')
    parser.add_argument('export_dir', help='导出数据目录路径')
    parser.add_argument('--config', help='配置文件路径（可选）')
    parser.add_argument('--root-strategy', choices=['center', 'max_degree', 'manual'], 
                       default='center', help='根节点选择策略 (默认: center)')
    parser.add_argument('--manual-root', help='手动指定的根节点名称 (当策略为 manual 时使用)')
    parser.add_argument('--with-actuators', action='store_true', 
                       help='生成带执行器的模型文件 (model-actuator-position.xml)')
    
    args = parser.parse_args()
    
    # 加载配置
    config = {}
    if args.config:
        with open(args.config, 'r', encoding='utf-8') as f:
            config = json.load(f)
    
    # 添加命令行参数到配置
    if args.root_strategy:
        config['root_node_strategy'] = args.root_strategy
    if args.manual_root:
        config['manual_root_node'] = args.manual_root
    
    # 添加命令行参数到配置
    if args.with_actuators:
        config['with_actuators'] = True
        logger.info("⚡ 将生成带执行器的模型文件")
    else:
        config['with_actuators'] = False
        logger.info("📋 生成基础模型文件")
    
    # 创建转换器并执行转换
    converter = MaojocoConverter(config)
    result = converter.convert(args.export_dir)
    
    # 输出结果
    if result.success:
        logger.success(f"转换成功: {result.message}")
        logger.info(f"📊 结果数据: {result.data}")
    else:
        logger.error_msg(f"转换失败: {result.message}")
        exit(1)


if __name__ == "__main__":
    main()