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
except ImportError:
    # 如果相对导入失败，尝试绝对导入
    sys.path.append(os.path.dirname(os.path.dirname(os.path.abspath(__file__))))
    from core.motion_manager import get_motion_manager
    from core.motion_sequence import Keyframe, MotionSequence
    from gui.pose_save_dialog import show_save_pose_dialog


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
    
    def __init__(self, parent=None):
        super().__init__("🎬 动作编辑器", parent)
        self.motion_manager = get_motion_manager()
        self.current_sequence = None
        self.is_playing = False
        self.current_time = 0.0
        self.timer = QTimer()
        self.timer.setInterval(50)  # 20fps
        self.timer.timeout.connect(self.update_playback_progress)
        
        self.init_ui()
        self.setup_connections()
        self.load_sequence_list()
        
    def init_ui(self):
        """初始化UI"""
        layout = QVBoxLayout()
        layout.setSpacing(8)
        
        # 左侧区域：序列和关键帧
        left_layout = QVBoxLayout()
        
        # 动作序列区域
        sequence_group = QGroupBox("🎬 动作序列")
        sequence_layout = QVBoxLayout()
        
        # 序列操作按钮
        seq_btn_layout = QHBoxLayout()
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
        
        # 序列列表
        self.sequence_list = QListWidget()
        self.sequence_list.setMinimumHeight(100)
        self.sequence_list.setMaximumHeight(150)
        self.sequence_list.setStyleSheet("""
            QListWidget {
                border: 1px solid #ddd;
                border-radius: 4px;
                background-color: #f9f9f9;
                padding: 5px;
            }
            QListWidget::item {
                padding: 6px;
                border-bottom: 1px solid #eee;
            }
            QListWidget::item:selected {
                background-color: #e3f2fd;
                color: #1976d2;
            }
        """)
        sequence_layout.addWidget(self.sequence_list)
        sequence_group.setLayout(sequence_layout)
        left_layout.addWidget(sequence_group)
        
        # 关键帧区域
        keyframe_group = QGroupBox("🎯 关键帧")
        keyframe_layout = QVBoxLayout()
        
        # 关键帧操作按钮
        key_btn_layout = QHBoxLayout()
        self.add_key_btn = QPushButton("➕ 添加")
        self.add_key_btn.clicked.connect(self.add_keyframe)
        self.edit_key_btn = QPushButton("✏️ 编辑")
        self.edit_key_btn.clicked.connect(self.edit_keyframe)
        self.delete_key_btn = QPushButton("🗑️ 删除")
        self.delete_key_btn.clicked.connect(self.delete_keyframe)
        
        key_btn_layout.addWidget(self.add_key_btn)
        key_btn_layout.addWidget(self.edit_key_btn)
        key_btn_layout.addWidget(self.delete_key_btn)
        keyframe_layout.addLayout(key_btn_layout)
        
        # 关键帧列表
        self.keyframe_list = QListWidget()
        self.keyframe_list.setMinimumHeight(100)
        self.keyframe_list.setMaximumHeight(150)
        self.keyframe_list.setStyleSheet("""
            QListWidget {
                border: 1px solid #ddd;
                border-radius: 4px;
                background-color: #f9f9f9;
                padding: 5px;
            }
            QListWidget::item {
                padding: 6px;
                border-bottom: 1px solid #eee;
            }
            QListWidget::item:selected {
                background-color: #e3f2fd;
                color: #1976d2;
            }
        """)
        keyframe_layout.addWidget(self.keyframe_list)
        keyframe_group.setLayout(keyframe_layout)
        left_layout.addWidget(keyframe_group)
        
        # 右侧区域：播放控制
        right_layout = QVBoxLayout()
        
        # 播放控制区域
        playback_group = QGroupBox("🎮 播放控制")
        playback_layout = QVBoxLayout()
        
        # 播放按钮
        play_btn_layout = QHBoxLayout()
        self.play_btn = QPushButton("▶️ 播放")
        self.play_btn.clicked.connect(self.toggle_playback)
        self.stop_btn = QPushButton("⏹️ 停止")
        self.stop_btn.clicked.connect(self.stop_playback)
        
        play_btn_layout.addWidget(self.play_btn)
        play_btn_layout.addWidget(self.stop_btn)
        play_btn_layout.addStretch()
        playback_layout.addLayout(play_btn_layout)
        
        # 进度显示
        self.progress_label = QLabel("时间: 0.0s / 0.0s")
        self.progress_label.setStyleSheet("font-size: 12px; color: #666;")
        playback_layout.addWidget(self.progress_label)
        
        self.progress_bar = QProgressBar()
        self.progress_bar.setRange(0, 100)
        self.progress_bar.setValue(0)
        playback_layout.addWidget(self.progress_bar)
        
        # 速度控制
        speed_layout = QHBoxLayout()
        speed_label = QLabel("播放速度:")
        speed_layout.addWidget(speed_label)
        
        self.speed_combo = QComboBox()
        self.speed_combo.addItems(["0.5x", "1.0x", "1.5x", "2.0x"])
        self.speed_combo.setCurrentText("1.0x")
        speed_layout.addWidget(self.speed_combo)
        speed_layout.addStretch()
        
        playback_layout.addLayout(speed_layout)
        playback_group.setLayout(playback_layout)
        right_layout.addWidget(playback_group)
        
        right_layout.addStretch()
        
        # 组合左右布局
        content_layout = QHBoxLayout()
        content_layout.addWidget(QWidget(layout=left_layout), stretch=3)
        content_layout.addWidget(QWidget(layout=right_layout), stretch=2)
        
        layout.addLayout(content_layout)
        self.setLayout(layout)
    
    def setup_connections(self):
        """设置信号连接"""
        self.sequence_list.itemSelectionChanged.connect(self.on_sequence_selection_changed)
        self.sequence_list.itemDoubleClicked.connect(self.on_sequence_double_clicked)
        self.keyframe_list.itemSelectionChanged.connect(self.on_keyframe_selection_changed)
    
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
        existing_names = list(self.motion_manager.get_sequence_names())
        result = show_save_pose_dialog({}, existing_names, self, "动作序列")
        
        if result:
            sequence = MotionSequence(result['name'], [])
            keyframe = Keyframe('default_pose', 0.5, 1.0)
            sequence.add_keyframe(keyframe)
            
            success = self.motion_manager.create_sequence(result['name'], sequence.keyframes)
            
            if success:
                self.load_sequence_list()
                self.sequenceCreated.emit(result['name'])
                print(f"✅ 创建动作序列: {result['name']}")
            else:
                QMessageBox.warning(self, "创建失败", f"无法创建动作序列: {result['name']}")
    
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
                self.update_keyframe_list()
                self.progress_bar.setValue(0)
                self.progress_label.setText(f"时间: 0.0s / {sequence.total_duration:.1f}s")
                self.sequenceSelected.emit(seq_name)
                print(f"📁 加载动作序列: {seq_name}")
            else:
                print(f"❌ 无法加载动作序列: {seq_name}")
        except Exception as e:
            print(f"❌ 加载动作序列失败: {e}")
    
    def update_keyframe_list(self):
        """更新关键帧列表"""
        self.keyframe_list.clear()
        
        if not self.current_sequence:
            return
        
        for i, keyframe in enumerate(self.current_sequence.keyframes):
            timestamp = sum(kf.total_duration for kf in self.current_sequence.keyframes[:i])
            transition = keyframe.transition_duration
            hold = keyframe.hold_duration
            
            item_text = f"帧{i+1} ({timestamp:.1f}s) - {keyframe.pose_name}"
            item = QListWidgetItem(item_text)
            item.setData(Qt.UserRole, i)
            item.setToolTip(f"时间戳: {timestamp:.1f}s\n过渡时长: {transition:.1f}s\n保持时长: {hold:.1f}s\n姿态名称: {keyframe.pose_name}")
            
            self.keyframe_list.addItem(item)
    
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
            self.update_keyframe_list()
            self.load_sequence_list()  # 刷新序列列表显示的时长
            last_row = self.keyframe_list.count() - 1
            if last_row >= 0:
                self.keyframe_list.setCurrentRow(last_row)
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
                    self.update_keyframe_list()
                    self.load_sequence_list()  # 刷新序列列表显示的时长
                    self.keyframeDeleted.emit(keyframe_index)
                    print(f"🗑️ 删除关键帧: 第{keyframe_index+1}帧")
                else:
                    QMessageBox.warning(self, "删除失败", "无法删除关键帧")
    
    def on_keyframe_selection_changed(self):
        """关键帧选择改变处理"""
        current_item = self.keyframe_list.currentItem()
        if current_item:
            keyframe_index = current_item.data(Qt.UserRole)
            self.keyframeSelected.emit(keyframe_index)
    
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
        
        self.is_playing = True
        self.play_btn.setText("⏸️ 暂停")
        self.timer.start()
        self.playbackStarted.emit()
        print("▶️ 开始播放")
    
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
        self.update_progress_display()
        self.playbackStopped.emit()
        print("⏹️ 停止播放")
    
    def update_playback_progress(self):
        """更新播放进度"""
        if self.is_playing and self.current_sequence:
            speed = float(self.speed_combo.currentText().replace("x", ""))
            self.current_time += 0.05 * speed
            
            if self.current_time >= self.current_sequence.total_duration:
                self.current_time = self.current_sequence.total_duration
                self.stop_playback()
            else:
                self.update_progress_display()
                self.playbackProgress.emit(self.current_time)
    
    def update_progress_display(self):
        """更新进度显示"""
        if self.current_sequence:
            progress = (self.current_time / self.current_sequence.total_duration * 100) if self.current_sequence.total_duration > 0 else 0
            self.progress_bar.setValue(int(progress))
            self.progress_label.setText(f"时间: {self.current_time:.1f}s / {self.current_sequence.total_duration:.1f}s")


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