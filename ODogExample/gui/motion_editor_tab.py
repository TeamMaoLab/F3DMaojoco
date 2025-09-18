"""
ODogExample GUI模块 - 动作编辑器Tab页组件

独立的动作编辑器Tab页，包含完整的动作编辑功能。
"""

import sys
import os
from typing import Optional
from PySide6.QtWidgets import (
    QWidget, QVBoxLayout, QHBoxLayout, QGroupBox, 
    QPushButton, QLabel, QListWidget, QListWidgetItem,
    QComboBox, QProgressBar, QMessageBox, QFrame
)
from PySide6.QtCore import Signal, Qt, QTimer

try:
    from ..core.motion_manager import get_motion_manager
    from ..core.motion_sequence import Keyframe, MotionSequence
    from .pose_save_dialog import show_save_pose_dialog
    from ..pose_manager import get_pose_manager
except ImportError:
    # 如果相对导入失败，尝试绝对导入
    project_root = os.path.dirname(os.path.dirname(os.path.abspath(__file__)))
    if project_root not in sys.path:
        sys.path.insert(0, project_root)
    from core.motion_manager import get_motion_manager
    from core.motion_sequence import Keyframe, MotionSequence
    from gui.pose_save_dialog import show_save_pose_dialog
    from gui.pose_manager import get_pose_manager


class MotionSequenceTabWidget(QGroupBox):
    """动作序列Tab页组件"""
    
    # 信号定义
    sequenceSelected = Signal(str)      # 选中动作序列
    sequenceCreated = Signal(str)       # 创建动作序列
    sequenceDeleted = Signal(str)      # 删除动作序列
    keyframeSelected = Signal(int)     # 选中关键帧
    keyframeAdded = Signal(int)        # 添加关键帧
    keyframeDeleted = Signal(int)       # 删除关键帧
    playbackStarted = Signal()         # 播放开始
    playbackPaused = Signal()          # 播放暂停
    playbackStopped = Signal()         # 播放停止
    playbackProgress = Signal(float)   # 播放进度
    
    # 新增信号用于实际的机器人控制
    applyPoseRequest = Signal(str)     # 请求应用姿态
    applySequenceRequest = Signal(str) # 请求应用序列
    
    def __init__(self, parent=None):
        super().__init__("🎬 动作编辑器", parent)
        
        # 初始化管理器
        try:
            self.motion_manager = get_motion_manager()
            self.pose_manager = get_pose_manager()
            print(f"✅ 动作编辑器初始化成功")
            print(f"📋 可用姿态: {self.pose_manager.get_pose_names()}")
        except Exception as e:
            print(f"❌ 管理器初始化失败: {e}")
            self.motion_manager = None
            self.pose_manager = None
            
        self.current_sequence = None
        self.is_playing = False
        self.current_time = 0.0
        self.current_pose_index = 0  # 当前播放到的姿态索引
        self.pose_start_time = 0.0   # 当前姿态开始时间
        self.timer = QTimer()
        self.timer.setInterval(50)  # 20fps
        self.timer.timeout.connect(self.update_playback_progress)
        
        self.init_ui()
        self.setup_connections()
        self.load_sequence_list()
        
    def init_ui(self):
        """初始化UI"""
        main_layout = QVBoxLayout()
        main_layout.setSpacing(8)
        
        # 上方区域：动作序列和关键帧（合为一列）
        top_layout = QVBoxLayout()
        top_layout.setSpacing(8)
        
        # 动作序列（紧凑型）
        sequence_group = QGroupBox("🎬 动作序列")
        sequence_layout = QVBoxLayout()
        sequence_layout.setSpacing(3)
        sequence_layout.setContentsMargins(5, 8, 5, 5)
        
        # 序列操作按钮（紧凑）
        seq_btn_layout = QHBoxLayout()
        seq_btn_layout.setSpacing(5)
        self.create_seq_btn = QPushButton("➕ 新建")
        self.create_seq_btn.clicked.connect(self.create_sequence)
        self.delete_seq_btn = QPushButton("🗑️ 删除")
        self.delete_seq_btn.clicked.connect(self.delete_sequence)
        self.refresh_seq_btn = QPushButton("🔄 刷新")
        self.refresh_seq_btn.clicked.connect(self.refresh_sequence_list)
        
        seq_btn_layout.addWidget(self.create_seq_btn)
        seq_btn_layout.addWidget(self.delete_seq_btn)
        seq_btn_layout.addWidget(self.refresh_seq_btn)
        sequence_layout.addLayout(seq_btn_layout)
        
        # 序列列表（小一点）
        self.sequence_list = QListWidget()
        self.sequence_list.setMaximumHeight(70)  # 进一步限制高度
        self.sequence_list.setMinimumHeight(60)  # 设置最小高度
        self.sequence_list.setStyleSheet("""
            QListWidget {
                border: 1px solid #ddd;
                border-radius: 4px;
                background-color: #f9f9f9;
                padding: 2px;
                font-size: 11px;
            }
            QListWidget::item {
                padding: 2px;
                border-bottom: 1px solid #eee;
                min-height: 16px;
            }
            QListWidget::item:selected {
                background-color: #e3f2fd;
                color: #1976d2;
            }
        """)
        sequence_layout.addWidget(self.sequence_list)
        sequence_group.setLayout(sequence_layout)
        sequence_group.setMaximumHeight(sequence_group.sizeHint().height() + 10)
        top_layout.addWidget(sequence_group)
        
        # 姿态序列（主要区域）
        poses_group = QGroupBox("🎯 姿态序列")
        poses_layout = QVBoxLayout()
        poses_layout.setSpacing(5)
        poses_layout.setContentsMargins(5, 8, 5, 5)
        
        # 姿态操作按钮
        pose_btn_layout = QHBoxLayout()
        pose_btn_layout.setSpacing(5)
        self.add_pose_btn = QPushButton("➕ 添加姿态")
        self.add_pose_btn.clicked.connect(self.add_pose_to_sequence)
        self.edit_pose_btn = QPushButton("✏️ 编辑时长")
        self.edit_pose_btn.clicked.connect(self.edit_pose_duration)
        self.remove_pose_btn = QPushButton("🗑️ 移除")
        self.remove_pose_btn.clicked.connect(self.remove_pose_from_sequence)
        
        pose_btn_layout.addWidget(self.add_pose_btn)
        pose_btn_layout.addWidget(self.edit_pose_btn)
        pose_btn_layout.addWidget(self.remove_pose_btn)
        poses_layout.addLayout(pose_btn_layout)
        
        # 姿态列表（大一点）
        self.pose_list = QListWidget()
        self.pose_list.setMinimumHeight(150)  # 减小最小高度
        self.pose_list.setStyleSheet("""
            QListWidget {
                border: 1px solid #ddd;
                border-radius: 4px;
                background-color: #f9f9f9;
                padding: 3px;
            }
            QListWidget::item {
                padding: 4px;
                border-bottom: 1px solid #eee;
                min-height: 32px;
            }
            QListWidget::item:selected {
                background-color: #e3f2fd;
                color: #1976d2;
            }
        """)
        poses_layout.addWidget(self.pose_list)
        poses_group.setLayout(poses_layout)
        top_layout.addWidget(poses_group)
        
        main_layout.addLayout(top_layout)
        
        # 下方区域：播放控制和信息显示（紧凑布局）
        bottom_layout = QHBoxLayout()
        bottom_layout.setSpacing(8)
        
        # 左下：播放控制（紧凑）
        playback_group = QGroupBox("🎮 播放控制")
        playback_layout = QVBoxLayout()
        playback_layout.setSpacing(5)
        playback_layout.setContentsMargins(5, 8, 5, 5)
        
        # 播放按钮（水平布局）
        play_btn_layout = QHBoxLayout()
        play_btn_layout.setSpacing(5)
        self.play_btn = QPushButton("▶️ 播放")
        self.play_btn.clicked.connect(self.toggle_playback)
        self.stop_btn = QPushButton("⏹️ 停止")
        self.stop_btn.clicked.connect(self.stop_playback)
        
        play_btn_layout.addWidget(self.play_btn)
        play_btn_layout.addWidget(self.stop_btn)
        play_btn_layout.addStretch()
        playback_layout.addLayout(play_btn_layout)
        
        # 进度显示（紧凑）
        self.progress_label = QLabel("时间: 0.0s / 0.0s")
        self.progress_label.setStyleSheet("font-size: 11px; color: #666;")
        playback_layout.addWidget(self.progress_label)
        
        self.progress_bar = QProgressBar()
        self.progress_bar.setRange(0, 100)
        self.progress_bar.setValue(0)
        self.progress_bar.setMaximumHeight(15)  # 限制进度条高度
        playback_layout.addWidget(self.progress_bar)
        
        # 速度控制（紧凑）
        speed_layout = QHBoxLayout()
        speed_layout.setSpacing(5)
        speed_label = QLabel("速度:")
        speed_label.setStyleSheet("font-size: 11px;")
        speed_layout.addWidget(speed_label)
        
        self.speed_combo = QComboBox()
        self.speed_combo.addItems(["0.5x", "1.0x", "1.5x", "2.0x"])
        self.speed_combo.setCurrentText("1.0x")
        speed_layout.addWidget(self.speed_combo)
        speed_layout.addStretch()
        
        playback_layout.addLayout(speed_layout)
        playback_group.setLayout(playback_layout)
        playback_group.setMaximumHeight(playback_group.sizeHint().height() + 10)
        bottom_layout.addWidget(playback_group, stretch=1)
        
        # 右下：信息显示（紧凑）
        info_group = QGroupBox("📊 序列信息")
        info_layout = QVBoxLayout()
        info_layout.setSpacing(5)
        info_layout.setContentsMargins(5, 8, 5, 5)
        
        # 序列信息显示
        self.info_label = QLabel("未选择序列")
        self.info_label.setWordWrap(True)
        self.info_label.setStyleSheet("color: #666; font-size: 11px;")
        info_layout.addWidget(self.info_label)
        
        # 当前关键帧信息
        self.keyframe_info_label = QLabel("未选择关键帧")
        self.keyframe_info_label.setWordWrap(True)
        self.keyframe_info_label.setStyleSheet("color: #666; font-size: 11px;")
        info_layout.addWidget(self.keyframe_info_label)
        
        info_group.setLayout(info_layout)
        info_group.setMaximumHeight(info_group.sizeHint().height() + 10)
        bottom_layout.addWidget(info_group, stretch=1)
        
        main_layout.addLayout(bottom_layout)
        self.setLayout(main_layout)
    
    def setup_connections(self):
        """设置信号连接"""
        self.sequence_list.itemSelectionChanged.connect(self.on_sequence_selection_changed)
        self.sequence_list.itemDoubleClicked.connect(self.on_sequence_double_clicked)
        self.pose_list.itemSelectionChanged.connect(self.on_pose_selection_changed)
        self.pose_list.itemDoubleClicked.connect(self.on_pose_double_clicked)
    
    def load_sequence_list(self):
        """加载动作序列列表"""
        try:
            sequences = self.motion_manager.get_all_sequences()
            self.sequence_list.clear()
            
            for seq_name, seq_data in sequences.items():
                duration = seq_data.total_duration
                keyframe_count = len(seq_data.keyframes)
                
                item_text = f"{seq_name} ({duration:.1f}s, {keyframe_count}帧)"
                item = QListWidgetItem(item_text)
                item.setData(Qt.UserRole, seq_name)
                item.setToolTip(f"时长: {duration:.1f}秒\n关键帧数: {keyframe_count}")
                
                self.sequence_list.addItem(item)
            
        except Exception as e:
            print(f"❌ 加载动作序列列表失败: {e}")
    
    def create_sequence(self):
        """创建新的动作序列"""
        from PySide6.QtWidgets import QDialog, QVBoxLayout, QLabel, QLineEdit, QDialogButtonBox
        
        dialog = QDialog(self)
        dialog.setWindowTitle("创建动作序列")
        dialog.setFixedSize(300, 150)
        
        layout = QVBoxLayout()
        layout.setSpacing(15)
        layout.setContentsMargins(20, 20, 20, 20)
        
        # 序列名称输入
        name_label = QLabel("序列名称:")
        name_label.setStyleSheet("font-weight: bold;")
        name_edit = QLineEdit()
        name_edit.setPlaceholderText("请输入序列名称...")
        
        layout.addWidget(name_label)
        layout.addWidget(name_edit)
        
        # 按钮
        buttons = QDialogButtonBox(QDialogButtonBox.Ok | QDialogButtonBox.Cancel)
        buttons.accepted.connect(dialog.accept)
        buttons.rejected.connect(dialog.reject)
        layout.addWidget(buttons)
        
        dialog.setLayout(layout)
        name_edit.setFocus()
        
        if dialog.exec() == QDialog.Accepted:
            sequence_name = name_edit.text().strip()
            
            if not sequence_name:
                QMessageBox.warning(self, "输入错误", "请输入序列名称！")
                return
            
            # 检查名称是否已存在
            existing_names = list(self.motion_manager.get_sequence_names())
            if sequence_name in existing_names:
                QMessageBox.warning(self, "名称重复", f"序列名称 '{sequence_name}' 已存在！")
                return
            
            # 创建序列，默认添加IDLE姿态作为第一个姿态
            try:
                from ..core.motion_sequence import Keyframe
            except ImportError:
                from core.motion_sequence import Keyframe
            idle_keyframe = Keyframe("IDLE", 0.5, 1.0, "linear")
            sequence = MotionSequence(sequence_name, [idle_keyframe])
            
            success = self.motion_manager.create_sequence(sequence_name, sequence.keyframes)
            
            if success:
                self.load_sequence_list()
                self.sequenceCreated.emit(sequence_name)
                print(f"✅ 创建动作序列: {sequence_name}")
                
                # 自动选中新创建的序列
                items = self.sequence_list.findItems(sequence_name, Qt.MatchExactly)
                if items:
                    self.sequence_list.setCurrentItem(items[0])
                    self.load_sequence(sequence_name)
                    
                # 提示用户序列已创建
                QMessageBox.information(self, "创建成功", 
                    f"动作序列 '{sequence_name}' 创建成功！\n\n"
                    f"序列已包含 IDLE 姿态作为起始状态。\n"
                    f"你可以继续添加更多姿态来丰富动作序列。")
            else:
                QMessageBox.warning(self, "创建失败", f"无法创建动作序列: {sequence_name}")
    
    def delete_sequence(self):
        """删除选中的动作序列"""
        current_item = self.sequence_list.currentItem()
        if not current_item:
            QMessageBox.information(self, "提示", "请先选择要删除的动作序列！")
            return
        
        seq_name = current_item.data(Qt.UserRole)
        
        reply = QMessageBox.question(
            self, "确认删除", 
            f"确定要删除动作序列 '{seq_name}' 吗？\n此操作不可恢复！",
            QMessageBox.Yes | QMessageBox.No,
            QMessageBox.No
        )
        
        if reply == QMessageBox.Yes:
            success = self.motion_manager.delete_sequence(seq_name)
            if success:
                self.load_sequence_list()
                self.sequenceDeleted.emit(seq_name)
                print(f"🗑️ 删除动作序列: {seq_name}")
            else:
                QMessageBox.warning(self, "删除失败", f"无法删除动作序列: {seq_name}")
    
    def refresh_sequence_list(self):
        """刷新序列列表"""
        self.load_sequence_list()
    
    def on_sequence_selection_changed(self):
        """序列选择改变处理"""
        current_item = self.sequence_list.currentItem()
        if current_item:
            seq_name = current_item.data(Qt.UserRole)
            self.load_sequence(seq_name)
    
    def on_sequence_double_clicked(self, item):
        """序列双击处理"""
        seq_name = item.data(Qt.UserRole)
        self.load_sequence(seq_name)
    
    def load_sequence(self, seq_name: str):
        """加载动作序列"""
        try:
            sequence = self.motion_manager.get_sequence(seq_name)
            if sequence:
                self.current_sequence = sequence
                # 强制开启循环播放
                self.current_sequence.loop = True
                self.update_pose_list()
                self.update_sequence_info()
                self.progress_bar.setValue(0)
                self.progress_label.setText(f"时间: 0.0s / {sequence.total_duration:.1f}s")
                self.sequenceSelected.emit(seq_name)
                print(f"📁 加载动作序列: {seq_name}")
                print(f"🔁 循环播放: 开启")
            else:
                print(f"❌ 无法加载动作序列: {seq_name}")
        except Exception as e:
            print(f"❌ 加载动作序列失败: {e}")
    
    def update_pose_list(self):
        """更新姿态序列列表"""
        self.pose_list.clear()
        
        if not self.current_sequence:
            return
        
        for i, keyframe in enumerate(self.current_sequence.keyframes):
            timestamp = sum(kf.total_duration for kf in self.current_sequence.keyframes[:i])
            
            item_text = f"第{i+1}个姿态: {keyframe.pose_name}"
            item_text += f"\n保持 {keyframe.hold_duration:.1f}秒"
            
            item = QListWidgetItem(item_text)
            item.setData(Qt.UserRole, i)
            item.setToolTip(f"时间戳: {timestamp:.1f}s\n姿态名称: {keyframe.pose_name}\n保持时长: {keyframe.hold_duration:.1f}秒\n过渡时长: {keyframe.transition_duration:.1f}秒")
            
            self.pose_list.addItem(item)
    
    def add_pose_to_sequence(self):
        """添加姿态到序列"""
        if not self.current_sequence:
            QMessageBox.warning(self, "警告", "请先选择一个动作序列！")
            return
        
        # 获取当前可用的姿态列表
        try:
            # 检查姿态管理器是否可用
            if not self.pose_manager:
                QMessageBox.critical(self, "错误", "姿态管理器未初始化！\n请重新启动应用。")
                return
            
            # 从姿态管理器获取可用姿态
            available_poses = self.pose_manager.get_pose_names()
            print(f"📦 从姿态管理器加载姿态列表: {available_poses}")
            
            if not available_poses:
                QMessageBox.warning(self, "警告", "姿态管理器中没有可用的姿态！\n请先在姿态编辑中创建一些姿态。")
                return
                
        except Exception as e:
            print(f"❌ 加载姿态列表失败: {e}")
            QMessageBox.critical(self, "错误", f"加载姿态列表失败: {e}")
            return
        
        # 创建姿态选择和时间设置对话框
        try:
            from PySide6.QtWidgets import (QDialog, QVBoxLayout, QListWidget, 
                                          QDialogButtonBox, QLabel, QDoubleSpinBox,
                                          QHBoxLayout, QComboBox)
            
            dialog = QDialog(self)
            dialog.setWindowTitle("添加姿态")
            dialog.setFixedSize(350, 450)
            
            layout = QVBoxLayout()
            layout.setSpacing(10)
            
            # 姿态选择
            pose_label = QLabel("选择姿态:")
            layout.addWidget(pose_label)
            
            pose_list = QListWidget()
            pose_list.addItems(available_poses)
            layout.addWidget(pose_list)
            
            # 过渡时长设置
            trans_layout = QHBoxLayout()
            trans_label = QLabel("过渡时长:")
            trans_spin = QDoubleSpinBox()
            trans_spin.setRange(0.1, 5.0)
            trans_spin.setSingleStep(0.1)
            trans_spin.setValue(0.5)
            trans_spin.setSuffix(" 秒")
            trans_layout.addWidget(trans_label)
            trans_layout.addWidget(trans_spin)
            trans_layout.addStretch()
            layout.addLayout(trans_layout)
            
            # 保持时长设置
            hold_layout = QHBoxLayout()
            hold_label = QLabel("保持时长:")
            hold_spin = QDoubleSpinBox()
            hold_spin.setRange(0.1, 10.0)
            hold_spin.setSingleStep(0.1)
            hold_spin.setValue(1.0)
            hold_spin.setSuffix(" 秒")
            hold_layout.addWidget(hold_label)
            hold_layout.addWidget(hold_spin)
            hold_layout.addStretch()
            layout.addLayout(hold_layout)
            
            # 插值类型设置
            interp_layout = QHBoxLayout()
            interp_label = QLabel("插值类型:")
            interp_combo = QComboBox()
            interp_combo.addItems(["linear", "smooth"])
            interp_combo.setCurrentText("linear")
            interp_layout.addWidget(interp_label)
            interp_layout.addWidget(interp_combo)
            interp_layout.addStretch()
            layout.addLayout(interp_layout)
            
            # 按钮
            buttons = QDialogButtonBox(QDialogButtonBox.Ok | QDialogButtonBox.Cancel)
            buttons.accepted.connect(dialog.accept)
            buttons.rejected.connect(dialog.reject)
            layout.addWidget(buttons)
            
            dialog.setLayout(layout)
            
            if dialog.exec() == QDialog.Accepted and pose_list.currentItem():
                pose_name = pose_list.currentItem().text()
                transition_duration = trans_spin.value()
                hold_duration = hold_spin.value()
                interpolation_type = interp_combo.currentText()
                
                # 创建新的关键帧（姿态）
                keyframe = Keyframe(pose_name, transition_duration, hold_duration, interpolation_type)
                self.current_sequence.keyframes.append(keyframe)
                
                # 更新序列
                success = self.motion_manager.update_sequence(self.current_sequence.name, self.current_sequence)
                if success:
                    self.update_pose_list()
                    self.load_sequence_list()  # 刷新序列列表显示的时长
                    last_row = self.pose_list.count() - 1
                    if last_row >= 0:
                        self.pose_list.setCurrentRow(last_row)
                    print(f"➕ 添加姿态: {pose_name} (过渡{transition_duration}s, 保持{hold_duration}s)")
                else:
                    QMessageBox.warning(self, "添加失败", "无法添加姿态到序列")
                    
        except Exception as e:
            print(f"❌ 添加姿态失败: {e}")
            QMessageBox.warning(self, "添加失败", f"无法添加姿态: {e}")
    
    def edit_pose_duration(self):
        """编辑姿态时长"""
        current_item = self.pose_list.currentItem()
        if not current_item:
            QMessageBox.information(self, "提示", "请先选择要编辑的姿态！")
            return
        
        pose_index = current_item.data(Qt.UserRole)
        if not (0 <= pose_index < len(self.current_sequence.keyframes)):
            return
        
        keyframe = self.current_sequence.keyframes[pose_index]
        
        # 创建时长编辑对话框
        from PySide6.QtWidgets import QDialog, QVBoxLayout, QLabel, QDoubleSpinBox, QDialogButtonBox
        
        dialog = QDialog(self)
        dialog.setWindowTitle("编辑姿态时长")
        dialog.setFixedSize(300, 200)
        
        layout = QVBoxLayout()
        
        # 过渡时长
        trans_label = QLabel("过渡时长（秒）:")
        trans_spin = QDoubleSpinBox()
        trans_spin.setRange(0.1, 5.0)
        trans_spin.setSingleStep(0.1)
        trans_spin.setValue(keyframe.transition_duration)
        layout.addWidget(trans_label)
        layout.addWidget(trans_spin)
        
        # 保持时长
        hold_label = QLabel("保持时长（秒）:")
        hold_spin = QDoubleSpinBox()
        hold_spin.setRange(0.1, 10.0)
        hold_spin.setSingleStep(0.1)
        hold_spin.setValue(keyframe.hold_duration)
        layout.addWidget(hold_label)
        layout.addWidget(hold_spin)
        
        buttons = QDialogButtonBox(QDialogButtonBox.Ok | QDialogButtonBox.Cancel)
        buttons.accepted.connect(dialog.accept)
        buttons.rejected.connect(dialog.reject)
        layout.addWidget(buttons)
        
        dialog.setLayout(layout)
        
        if dialog.exec() == QDialog.Accepted:
            keyframe.transition_duration = trans_spin.value()
            keyframe.hold_duration = hold_spin.value()
            
            # 更新序列
            success = self.motion_manager.update_sequence(self.current_sequence.name, self.current_sequence)
            if success:
                self.update_pose_list()
                self.load_sequence_list()  # 刷新序列列表显示的时长
                self.update_sequence_info()  # 更新序列信息
                print(f"✏️ 编辑姿态时长: {keyframe.pose_name}")
            else:
                QMessageBox.warning(self, "编辑失败", "无法保存时长修改")
    
    def remove_pose_from_sequence(self):
        """从序列中移除姿态"""
        current_item = self.pose_list.currentItem()
        if not current_item:
            QMessageBox.information(self, "提示", "请先选择要移除的姿态！")
            return
        
        pose_index = current_item.data(Qt.UserRole)
        
        reply = QMessageBox.question(
            self, "确认移除", 
            f"确定要移除第{pose_index+1}个姿态吗？",
            QMessageBox.Yes | QMessageBox.No,
            QMessageBox.No
        )
        
        if reply == QMessageBox.Yes:
            if 0 <= pose_index < len(self.current_sequence.keyframes):
                removed_pose = self.current_sequence.keyframes.pop(pose_index)
                success = self.motion_manager.update_sequence(self.current_sequence.name, self.current_sequence)
                if success:
                    self.update_pose_list()
                    self.load_sequence_list()  # 刷新序列列表显示的时长
                    self.update_sequence_info()  # 更新序列信息
                    print(f"🗑️ 移除姿态: {removed_pose.pose_name}")
                else:
                    QMessageBox.warning(self, "移除失败", "无法移除姿态")
    
    def on_pose_selection_changed(self):
        """姿态选择改变处理"""
        current_item = self.pose_list.currentItem()
        if current_item:
            pose_index = current_item.data(Qt.UserRole)
            self.update_pose_info(pose_index)
            self.keyframeSelected.emit(pose_index)  # 复用原有的信号
        else:
            self.keyframe_info_label.setText("未选择姿态")
    
    def on_pose_double_clicked(self, item):
        """姿态双击处理"""
        pose_index = item.data(Qt.UserRole)
        self.edit_pose_duration()  # 双击直接编辑时长
    
    def update_pose_info(self, pose_index: int):
        """更新姿态信息显示"""
        if self.current_sequence and 0 <= pose_index < len(self.current_sequence.keyframes):
            keyframe = self.current_sequence.keyframes[pose_index]
            timestamp = sum(kf.total_duration for kf in self.current_sequence.keyframes[:pose_index])
            
            info_text = f"姿态: 第{pose_index+1}个\n"
            info_text += f"时间戳: {timestamp:.1f}秒\n"
            info_text += f"姿态名称: {keyframe.pose_name}\n"
            info_text += f"过渡时长: {keyframe.transition_duration:.1f}秒\n"
            info_text += f"保持时长: {keyframe.hold_duration:.1f}秒\n"
            info_text += f"插值类型: {keyframe.interpolation_type}"
            self.keyframe_info_label.setText(info_text)
        else:
            self.keyframe_info_label.setText("未选择姿态")
    
    def add_keyframe(self):
        """添加关键帧"""
        if not self.current_sequence:
            QMessageBox.warning(self, "警告", "请先选择一个动作序列！")
            return
        
        keyframe = Keyframe('new_pose', 0.5, 1.0)
        self.current_sequence.keyframes.append(keyframe)
        
        # 更新序列
        success = self.motion_manager.update_sequence(self.current_sequence.name, self.current_sequence)
        if success:
            self.update_pose_list()
            self.load_sequence_list()  # 刷新序列列表显示的时长
            last_row = self.pose_list.count() - 1
            if last_row >= 0:
                self.pose_list.setCurrentRow(last_row)
            self.keyframeAdded.emit(last_row)
            print(f"➕ 添加关键帧: 第{last_row+1}帧")
        else:
            QMessageBox.warning(self, "添加失败", "无法添加关键帧")
    
    def edit_keyframe(self):
        """编辑关键帧"""
        current_item = self.keyframe_list.currentItem()
        if not current_item:
            QMessageBox.information(self, "提示", "请先选择要编辑的关键帧！")
            return
        
        keyframe_index = current_item.data(Qt.UserRole)
        print(f"✏️ 编辑关键帧: 第{keyframe_index+1}帧")
        # 这里可以添加编辑对话框
    
    def delete_keyframe(self):
        """删除关键帧"""
        current_item = self.keyframe_list.currentItem()
        if not current_item:
            QMessageBox.information(self, "提示", "请先选择要删除的关键帧！")
            return
        
        keyframe_index = current_item.data(Qt.UserRole)
        
        reply = QMessageBox.question(
            self, "确认删除", 
            f"确定要删除第{keyframe_index+1}帧吗？",
            QMessageBox.Yes | QMessageBox.No,
            QMessageBox.No
        )
        
        if reply == QMessageBox.Yes:
            if keyframe_index < len(self.current_sequence.keyframes):
                self.current_sequence.keyframes.pop(keyframe_index)
                success = self.motion_manager.update_sequence(self.current_sequence.name, self.current_sequence)
                if success:
                    self.update_pose_list()
                    self.load_sequence_list()  # 刷新序列列表显示的时长
                    self.keyframeDeleted.emit(keyframe_index)
                    print(f"🗑️ 删除关键帧: 第{keyframe_index+1}帧")
                else:
                    QMessageBox.warning(self, "删除失败", "无法删除关键帧")
    
    def on_keyframe_selection_changed(self):
        """关键帧选择改变处理（已弃用，保留兼容性）"""
        # 这个方法已不再使用，保留以避免潜在的引用错误
        pass
    
    def toggle_playback(self):
        """切换播放状态"""
        if self.is_playing:
            self.pause_playback()
        else:
            self.start_playback()
    
    def start_playback(self):
        """开始播放"""
        if not self.current_sequence:
            QMessageBox.warning(self, "警告", "请先选择一个动作序列！")
            return
        
        if not self.current_sequence.keyframes:
            QMessageBox.warning(self, "警告", "当前序列没有姿态！")
            return
        
        print(f"▶️ 开始播放序列: {self.current_sequence.name}")
        print(f"📋 序列包含 {len(self.current_sequence.keyframes)} 个姿态")
        
        self.is_playing = True
        self.current_time = 0.0
        self.current_pose_index = 0
        self.pose_start_time = 0.0
        self.play_btn.setText("⏸️ 暂停")
        
        # 应用第一个姿态
        print("🎯 应用第一个姿态...")
        self.apply_current_pose()
        
        self.timer.start()
        self.playbackStarted.emit()
        print("▶️ 播放已开始，定时器启动")
    
    def pause_playback(self):
        """暂停播放"""
        self.is_playing = False
        self.play_btn.setText("▶️ 播放")
        self.timer.stop()
        self.playbackPaused.emit()
        print("⏸️ 暂停播放")
    
    def stop_playback(self):
        """停止播放"""
        self.is_playing = False
        self.play_btn.setText("▶️ 播放")
        self.timer.stop()
        self.current_time = 0.0
        self.current_pose_index = 0
        self.pose_start_time = 0.0
        self.update_progress_display()
        self.playbackStopped.emit()
        print("⏹️ 停止播放")
    
    def update_playback_progress(self):
        """更新播放进度"""
        if self.is_playing and self.current_sequence:
            speed = float(self.speed_combo.currentText().replace("x", ""))
            old_time = self.current_time
            self.current_time += 0.05 * speed
            
            # 检查是否需要切换到下一个姿态
            if self.current_pose_index < len(self.current_sequence.keyframes):
                current_keyframe = self.current_sequence.keyframes[self.current_pose_index]
                pose_duration = current_keyframe.total_duration
                
                elapsed_time = self.current_time - self.pose_start_time
                
                if elapsed_time >= pose_duration:
                    print(f"⏰ 姿态 {self.current_pose_index + 1} 播放完成 ({elapsed_time:.2f}s >= {pose_duration:.2f}s)")
                    # 切换到下一个姿态
                    self.current_pose_index += 1
                    self.pose_start_time = self.current_time
                    
                    if self.current_pose_index < len(self.current_sequence.keyframes):
                        print(f"🔄 切换到姿态 {self.current_pose_index + 1}")
                        self.apply_current_pose()
                    else:
                        # 序列播放完毕
                        print("🏁 序列播放完毕")
                        if self.current_sequence.loop:
                            print("🔄 循环播放")
                            # 循环播放
                            self.current_pose_index = 0
                            self.pose_start_time = 0.0
                            self.current_time = 0.0
                            self.apply_current_pose()
                        else:
                            # 停止播放
                            print("⏹️ 停止播放")
                            self.stop_playback()
                            return
            
            if self.current_time >= self.current_sequence.total_duration:
                print(f"⏰ 总时间达到: {self.current_time:.2f}s >= {self.current_sequence.total_duration:.2f}s")
                if self.current_sequence.loop:
                    print("🔄 循环播放（总时间）")
                    # 循环播放
                    self.current_time = 0.0
                    self.current_pose_index = 0
                    self.pose_start_time = 0.0
                    self.apply_current_pose()
                else:
                    self.current_time = self.current_sequence.total_duration
                    print("⏹️ 停止播放（总时间）")
                    self.stop_playback()
                    return
            
            # 每1秒打印一次进度
            if int(self.current_time) > int(old_time):
                print(f"⏱️ 播放进度: {self.current_time:.1f}s / {self.current_sequence.total_duration:.1f}s")
            
            self.update_progress_display()
            self.playbackProgress.emit(self.current_time)
    
    def apply_current_pose(self):
        """应用当前姿态"""
        if self.current_sequence and 0 <= self.current_pose_index < len(self.current_sequence.keyframes):
            keyframe = self.current_sequence.keyframes[self.current_pose_index]
            print(f"🎯 应用姿态: {keyframe.pose_name} (第{self.current_pose_index + 1}个)")
            print(f"📡 发送applyPoseRequest信号: {keyframe.pose_name}")
            self.applyPoseRequest.emit(keyframe.pose_name)
            print(f"✅ 信号已发送")
    
    def update_progress_display(self):
        """更新进度显示"""
        if self.current_sequence:
            progress = (self.current_time / self.current_sequence.total_duration * 100) if self.current_sequence.total_duration > 0 else 0
            self.progress_bar.setValue(int(progress))
            self.progress_label.setText(f"时间: {self.current_time:.1f}s / {self.current_sequence.total_duration:.1f}s")
    
    def update_sequence_info(self):
        """更新序列信息显示"""
        if self.current_sequence:
            info_text = f"序列名称: {self.current_sequence.name}\\n"
            info_text += f"关键帧数: {len(self.current_sequence.keyframes)}\\n"
            info_text += f"总时长: {self.current_sequence.total_duration:.1f}秒\\n"
            info_text += f"循环播放: {'是' if self.current_sequence.loop else '否'}"
            self.info_label.setText(info_text)
        else:
            self.info_label.setText("未选择序列")
    
        
    def update_keyframe_info(self, keyframe_index: int):
        """更新关键帧信息显示（已弃用，保留兼容性）"""
        # 这个方法已不再使用，保留以避免潜在的引用错误
        pass


if __name__ == "__main__":
    """测试脚本"""
    from PySide6.QtWidgets import QApplication
    
    app = QApplication(sys.argv)
    
    # 创建动作编辑器Tab页
    widget = MotionSequenceTabWidget()
    widget.setWindowTitle("ODogExample 动作编辑器Tab页测试")
    widget.resize(800, 600)
    
    # 测试信号连接
    widget.sequenceSelected.connect(lambda name: print(f"序列选中: {name}"))
    widget.sequenceCreated.connect(lambda name: print(f"序列创建: {name}"))
    widget.sequenceDeleted.connect(lambda name: print(f"序列删除: {name}"))
    widget.keyframeSelected.connect(lambda idx: print(f"关键帧选中: 第{idx+1}帧"))
    widget.keyframeAdded.connect(lambda idx: print(f"关键帧添加: 第{idx+1}帧"))
    widget.keyframeDeleted.connect(lambda idx: print(f"关键帧删除: 第{idx+1}帧"))
    widget.playbackStarted.connect(lambda: print("播放开始"))
    widget.playbackPaused.connect(lambda: print("播放暂停"))
    widget.playbackStopped.connect(lambda: print("播放停止"))
    
    widget.show()
    print("🎬 动作编辑器Tab页测试启动成功！")
    
    sys.exit(app.exec())