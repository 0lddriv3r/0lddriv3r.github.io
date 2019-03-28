---
layout: post
title:  jupyter-notebook-extension
date:   2019-03-29 00:07:56
comments: True
categories: Python
---

之前使用jupyter notebook一直觉得不是很顺手，最近才发现其实他是有很多插件的（其实也是最近用的比较多，用tensorflow学习机器学习的相关内容）。

# 安装插件管理器
优秀的系统应该设计为插件加载模式，方便扩大生态圈；而优秀的插件加载模式应该有一个**插件管理器**的插件。

## 首先安装插件管理器：
```
pip install jupyter_contrib_nbextensions
```

我的环境在执行这一步的过程中总是*超时*，估计原因为境外库速度慢所以超时，于是改用清华大学的库：
```
pip install -i https://pypi.tuna.tsinghua.edu.cn/simple jupyter_contrib_nbextensions
```

## 设置启用
再将`jupyter_contrib_nbextensions`插件管理器设置为启动jupyter notebook时自启用：
```
jupyter contrib nbextension install --user 
```

# 我的插件集
选了几个必要的插件供参考：

## 1. Hinterland
不管是之前用的VS还是ST都是支持输入过程中弹出菜单提示候选项的，但是jupyter这方面支持的不是很好，后来发现可以输入之后敲Tab键再提示，但是长期这样影响效率，而且最重要的是，敲击过程中自动弹出提示有助于思考，并且！模糊匹配很重要，没有谁记得清那么多的变量和接口！

但是由于之前的使用习惯都支持敲击Tab或者Enter来选中，但是Hinterland只支持Enter键。刚开始还觉得不是很友好，后来一想，这主要用于写Python的程序，本来Python也是通过缩进来标识程序的，如果这里使用Tab键可能会发生混淆（主要是指输入过程），所以这么设计还是有一定的道理。

所以先使用一段时间，如果操作习惯实在不适应，再看看能不能通过修改插件的配置文件或者直接修改插件来使用Tab键。

## 2. Table of Contents
虽然jupyter notebook非常适合脚本和文档同时写，但是编写文档的过程难免会频繁跳转目录，没有这个功能岂不浪费了Markdown的强大优势？

## 3. Variable Inspector
所有编码过程都需要回顾程序中变量类型和取值，而且这个插件非常契合Matlab和Rstudio的使用习惯（可能就是他们中迁移过来的用户自己动手的吧[chuckle]）。

## 4. ExecuteTime
机器学习中的训练时间开销是非常重要的用于衡量模型的指标，所以程序执行时间是经常用到的。

## 5. Autopep8
编码不规范，测试两行泪。

一键代码规范，人狠话不多。