"""
备份工具

提供文件备份功能，自动将非备份文件移动到备份文件夹中。
"""

import os
import shutil
import re
from datetime import datetime


class BackupManager:
    """备份管理器
    
    自动将非备份文件移动到备份文件夹中：
    - 识别 BAK_{YYMMDDHHmmss} 格式的备份文件
    - 创建带时间戳的备份文件夹
    - 移动非备份文件到备份文件夹
    """
    
    def __init__(self, logger=None):
        """初始化备份管理器
        
        Args:
            logger: 日志记录器
        """
        self.logger = logger
        self.backup_pattern = re.compile(r'^BAK_\d{12}$')
        self.exclude_files = set()  # 要排除的文件列表
    
    def add_exclude_file(self, filename: str):
        """添加要排除的文件
        
        Args:
            filename: 要排除的文件名
        """
        self.exclude_files.add(filename)
    
    def create_backup(self, directory: str) -> bool:
        """为指定目录创建备份
        
        Args:
            directory: 要备份的目录路径
            
        Returns:
            bool: 是否成功创建备份
        """
        try:
            if not os.path.exists(directory):
                if self.logger:
                    self.logger.warning(f"目录不存在，跳过备份: {directory}")
                return False
            
            # 获取目录中的所有文件和文件夹
            items = os.listdir(directory)
            if not items:
                if self.logger:
                    self.logger.info(f"目录为空，跳过备份: {directory}")
                return True
            
            # 过滤出需要备份的文件和文件夹
            items_to_backup = []
            for item in items:
                item_path = os.path.join(directory, item)
                if os.path.isfile(item_path) or os.path.isdir(item_path):
                    # 跳过备份文件、备份文件夹和排除的文件
                    if not self._is_backup_item(item) and item not in self.exclude_files:
                        items_to_backup.append(item)
            
            if not items_to_backup:
                if self.logger:
                    self.logger.info(f"没有需要备份的文件: {directory}")
                return True
            
            # 创建备份文件夹
            backup_folder = self._create_backup_folder(directory)
            if not backup_folder:
                return False
            
            # 移动文件到备份文件夹
            success_count = 0
            for item in items_to_backup:
                source_path = os.path.join(directory, item)
                backup_path = os.path.join(backup_folder, item)
                
                try:
                    if os.path.exists(backup_path):
                        # 如果备份文件夹中已存在同名文件，添加时间戳
                        name, ext = os.path.splitext(item)
                        timestamp = datetime.now().strftime("%H%M%S")
                        backup_path = os.path.join(backup_folder, f"{name}_{timestamp}{ext}")
                    
                    shutil.move(source_path, backup_path)
                    success_count += 1
                    
                    if self.logger:
                        self.logger.debug(f"已备份: {item} -> {os.path.basename(backup_folder)}")
                        
                except Exception as e:
                    if self.logger:
                        self.logger.error(f"备份文件失败 {item}: {str(e)}")
            
            if self.logger:
                self.logger.info(f"备份完成: {success_count}/{len(items_to_backup)} 个文件已备份到 {os.path.basename(backup_folder)}")
            
            return success_count > 0
            
        except Exception as e:
            if self.logger:
                self.logger.error(f"创建备份失败: {str(e)}")
            return False
    
    def _is_backup_item(self, item_name: str) -> bool:
        """检查是否为备份文件或文件夹
        
        Args:
            item_name: 文件或文件夹名称
            
        Returns:
            bool: 是否为备份项目
        """
        # 检查是否为备份文件格式 BAK_YYMMDDHHmmss
        if self.backup_pattern.match(item_name):
            return True
        
        # 检查是否为备份文件夹格式 backup_YYMMDDHHmmss
        if item_name.startswith('backup_') and len(item_name) == 19:
            try:
                # 尝试解析时间戳
                timestamp_str = item_name[7:19]
                datetime.strptime(timestamp_str, "%Y%m%d%H%M%S")
                return True
            except ValueError:
                pass
        
        return False
    
    def _create_backup_folder(self, directory: str) -> str:
        """创建备份文件夹
        
        Args:
            directory: 原始目录路径
            
        Returns:
            str: 备份文件夹路径，创建失败返回None
        """
        try:
            # 生成时间戳
            timestamp = datetime.now().strftime("%Y%m%d%H%M%S")
            backup_folder_name = f"backup_{timestamp}"
            backup_folder_path = os.path.join(directory, backup_folder_name)
            
            # 创建备份文件夹
            os.makedirs(backup_folder_path, exist_ok=True)
            
            if self.logger:
                self.logger.info(f"创建备份文件夹: {backup_folder_name}")
            
            return backup_folder_path
            
        except Exception as e:
            if self.logger:
                self.logger.error(f"创建备份文件夹失败: {str(e)}")
            return None
    
    def get_backup_info(self, directory: str) -> dict:
        """获取目录的备份信息
        
        Args:
            directory: 目录路径
            
        Returns:
            dict: 备份信息
        """
        try:
            if not os.path.exists(directory):
                return {"exists": False, "backups": []}
            
            backup_folders = []
            backup_files = []
            other_files = []
            
            for item in os.listdir(directory):
                item_path = os.path.join(directory, item)
                
                if self._is_backup_item(item):
                    if os.path.isdir(item_path):
                        backup_folders.append(item)
                    else:
                        backup_files.append(item)
                else:
                    other_files.append(item)
            
            return {
                "exists": True,
                "backup_folders": sorted(backup_folders, reverse=True),
                "backup_files": sorted(backup_files, reverse=True),
                "other_files": sorted(other_files),
                "backup_count": len(backup_folders) + len(backup_files),
                "other_count": len(other_files)
            }
            
        except Exception as e:
            if self.logger:
                self.logger.error(f"获取备份信息失败: {str(e)}")
            return {"exists": False, "error": str(e)}