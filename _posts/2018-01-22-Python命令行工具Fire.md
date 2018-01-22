---
layout: post
title:  Python命令行工具Fire
date:   2018-01-22 20:25:02
comments: True
categories: Python
---

今天发现了一个非常好用的python命令行工具[Fire](https://opensource.google.com/projects/python-fire)。
虽然python自带的开发库中有命令行生成工具，但是使用比较繁琐，失去了python简洁优雅的风格。同时，Fire还可以作为**调试工具**使用，更多请参见：[Python Fire Guide](https://google.github.io/python-fire/guide/)。

## 代码
非常的便捷，只需两行代码就能实现：
```python
import fire

def function(args):
    pass

if __name__ = '__main__':
    fire.Fire(function)
```
1. 如果python文件中只有一个函数，即使Fire()里不输入函数名，也能生成CLI。
2. 支持默认参数和类。

## 使用
在命令行输入`python ***.py -- --help`可以查看该文件的CLI，有两种方式使用：
1. 指定参数名：`pyhton ***.py --args args`。
2. 默认参数名：`python ***.py args`。