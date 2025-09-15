# 环境设置

### 解决 MuJoCo Python 动态库链接问题

在 macOS 系统中使用 `uv` 虚拟环境时，`mjpython` 可能因为找不到 `libpython3.x.dylib` 而无法启动。以下是完整的解决方案：

#### 前提条件
- 已通过 `uv` 创建虚拟环境（目录名为 `.venv`）
- 终端当前路径为项目根目录

#### 解决步骤

**1. 获取 Python 动态库信息**
```bash
# 获取 Python 库目录
PYTHON_LIB_DIR=$(python3 -c "import sysconfig; print(sysconfig.get_path('stdlib'))" | sed 's|/python3.[0-9]*||')

# 获取 Python 动态库名称  
PYTHON_LIB_NAME=$(python3 -c "import sys; print(f'libpython{sys.version_info.major}.{sys.version_info.minor}.dylib')")

echo "库目录: $PYTHON_LIB_DIR"
echo "库名称: $PYTHON_LIB_NAME"
```

**2. 创建软链接**
```bash
# 创建软链接到虚拟环境
ln -s "$PYTHON_LIB_DIR/$PYTHON_LIB_NAME" ./.venv/lib/$PYTHON_LIB_NAME
```

**3. 验证解决**
```bash
# 测试 mjpython 是否能正常启动
mjpython --version
```

#### 注意事项
- 如果重新创建 `uv` 虚拟环境，需要重新执行上述步骤
- 如果升级 Python 版本，动态库名称会发生变化，需要重新创建链接
- 如果遇到 "File exists" 错误，先删除旧链接：`rm ./.venv/lib/$PYTHON_LIB_NAME`

#### 示例输出
```
库目录: /Users/用户名/.local/share/uv/python/cpython-3.12.8-macos-aarch64-none/lib
库名称: libpython3.12.dylib
```

执行成功后，`.venv/lib/` 目录下会出现一个指向真实 Python 动态库的软链接。