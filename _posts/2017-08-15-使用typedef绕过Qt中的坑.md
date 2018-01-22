---
layout: post
title: 使用typedef绕过Qt中的坑
date: 2017-08-15 02:44:33
comment: True
categories: Qt
---

Qt的核心技术信号槽的灵活使用相信大家都非常熟悉了，这里主要总结一下在信号槽中不能随意使用的地方以及如何使用typedef绕过这些坑。
## 函数指针作为槽函数的参数
槽函数的使用和普通的成员函数没有多大的区别，但是不能将函数指针作为槽函数的参数！比如：
```c++
private slots:
void mySlot(int (*function)(char *p, int *n), bool b);
```
上述槽函数中使用了函数指针`int (*function)(char *p, int *n)`作为槽函数`mySlot`的第一个参数，这是不行的。如果非要使用函数指针作为参数应该如何通过编译呢？答案是使用`typedef`：
```c++
typedef int (*function)(char *p, int *n) FUNCTION(char *p, int *n)
...
private slots:
void mySlot(int FUNCTION(char *p, int *n), bool b);
```
## 模板类作为槽函数的参数
如果槽函数的参数含有模板类，即使编译的时候不报错，运行的时候也会产生错误。同样，使用`typedef`可以绕过：
```c++
typedef pair Pair;

public slots:
void mySlot(Pair myPair);
```