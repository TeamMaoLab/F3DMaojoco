# F3DMaojoco

F3D 到 MuJoCo 的转换工具，支持将 FreeCAD 中的 3D 模型转换为 MuJoCo 仿真环境。

## 文档结构

详细的文档请查看 `docs/` 目录：

- 📖 [环境设置](docs/setup/) - 系统配置和安装指南
- 📖 [使用指南](docs/guides/) - 详细的使用教程和示例
- 📖 [API 参考](docs/api/) - 开发接口文档

## 快速开始

1. 环境配置：参考 [macOS 环境设置](docs/setup/macos-setup.md)
2. 基础使用：查看 [使用指南](docs/guides/)
3. 开发参考：查阅 [API 文档](docs/api/)

## 项目结构

```
F3DMaojoco/
├── docs/                 # 文档目录
│   ├── setup/           # 环境设置
│   ├── guides/          # 使用指南
│   └── api/             # API 参考
├── MaojocoConverter/    # 核心转换器
├── F3DMaojocoScripts/   # 脚本工具
├── VistaQuickViewer/    # 快速查看器
└── tmp-output/          # 临时输出文件
```

## 许可证

参见 [LICENSE](LICENSE) 文件。