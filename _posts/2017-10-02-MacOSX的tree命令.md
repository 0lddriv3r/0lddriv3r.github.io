---
layout: post
title: MacOSX的tree命令
date: 2017-10-02 10:16:56
tags:
---

Linux和Windows下的tree命令对于需要查看一个目录的文件树非常有用，但是MacOS X中居然没有（Windows中都有，居然MacOS X中没有，这怎么能忍），找到一条shell命令可以实现这个功能需求：

```
find . -print | sed -e 's;[^/]*/;|____;g;s;____|; |;g'
```
效果如下：

```
➜  ScrapyProject git:(master) ✗ find . -print | sed -e 's;[^/]*/;|____;g;s;____|; |;g'
.
|____.DS_Store
|____.git
| |____branches
| |____config
| |____description
| |____HEAD
| |____hooks
| | |____applypatch-msg.sample
| | |____commit-msg.sample
| | |____post-update.sample
| | |____pre-applypatch.sample
| | |____pre-commit.sample
| | |____pre-push.sample
| | |____pre-rebase.sample
| | |____pre-receive.sample
| | |____prepare-commit-msg.sample
| | |____update.sample
| |____info
| | |____exclude
| |____objects
| | |____info
| | |____pack
| |____refs
| | |____heads
| | |____tags
|____scrapy.cfg
|____ScrapyProject
| |______init__.py
| |______init__.pyc
| |____items.py
| |____middlewares.py
| |____pipelines.py
| |____settings.py
| |____settings.pyc
| |____spiders
| | |______init__.py
| | |______init__.pyc
| | |____DmozSpider.py
```
每次这样使用也不方便，当然就想到了使用`alias`给命令设置别名：

```
alias tree="find . -print | sed -e 's;[^/]*/;|____;g;s;____|; |;g'"
```
**值得注意的是，需要在系统的.bashrc文件中添加该命令，这样才是全局有效并且永久生效（因为我用的是zsh，则在.zshrc文件中添加该命令）。**
现在即可使用`tree`命令查看当前目录下的文件树结构了：
```
➜  ScrapyProject git:(master) ✗ tree
.
|____.DS_Store
|____.git
| |____branches
| |____config
| |____description
| |____HEAD
| |____hooks
| | |____applypatch-msg.sample
| | |____commit-msg.sample
| | |____post-update.sample
| | |____pre-applypatch.sample
| | |____pre-commit.sample
| | |____pre-push.sample
| | |____pre-rebase.sample
| | |____pre-receive.sample
| | |____prepare-commit-msg.sample
| | |____update.sample
| |____info
| | |____exclude
| |____objects
| | |____info
| | |____pack
| |____refs
| | |____heads
| | |____tags
|____scrapy.cfg
|____ScrapyProject
| |______init__.py
| |______init__.pyc
| |____items.py
| |____middlewares.py
| |____pipelines.py
| |____settings.py
| |____settings.pyc
| |____spiders
| | |______init__.py
| | |______init__.pyc
| | |____DmozSpider.py
```