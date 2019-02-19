---
layout: post
title:  Cross-entropy Cost Function
date:   2019-02-19 21:43:02
comments: True
categories: Others
---

不要望文生义，会耽误很多功夫。这也是碎片化学习饱受诟病的地方：(

交叉熵代价函数跟信息论里的交叉熵没有什么关系，最多只是长得像而已...

维基了半天[交叉熵](https://zh.wikipedia.org/wiki/%E4%BA%A4%E5%8F%89%E7%86%B5)越看越糊涂，最后还是请教[朱博士](https://www.drifter.fun/)给我指点迷津。

# 概念由来
首先明确一个概念，交叉熵代价函数是为了解决ANN反向传播训练过程中，误差越大收敛速度应该越快的问题而反向**推导**出来的，最后大家看着这东西长得像交叉熵度量，所以取名**交叉熵代价函数**。

# 公式推导
## 二次代价函数失效
一般地，我们用二次代价函数度量预测值和实际值之间的误差：

$$ C = \frac{1}{2n} \sum_{x}{ ||y(x) - a(x)||^2 } $$

其中 $C$ 表示代价，$y$ 表示实际值，$a$表示预测值。

若采用梯度下降法进行调参训练，可得：

$$ \frac{\partial C}{\partial w} = (a-y)\sigma'(z)x $$

$$ \frac{\partial C}{\partial b} = (a-y)\sigma'(z) $$

其中 $z$表示神经元的输入，$\sigma$表示激活函数，在这里用的是[Sigmoid神经网络](https://0lddriv3r.github.io/math/2018/09/12/Sigmoid%E7%A5%9E%E7%BB%8F%E7%BD%91%E7%BB%9C.html)。由此可见，连接权重$w$和偏置量$b$与激活函数的梯度成正比，但是激活函数在误差越大的地方梯度是越小的，所以不能达到我们想要的*误差越大，调整越大，学习越快*的学习效果，也即是收敛速度。

## 反推交叉熵代价函数
要想达到与激活函数梯度无关的效果，可以通过反推的方法：

$$ \frac{\partial C}{\partial b} = \frac{\partial C}{\partial a}·\frac{\partial a}{\partial z}·\frac{\partial z}{\partial b} = \frac{\partial C}{\partial a}·\sigma'(z)·\frac{\partial wx+b}{\partial b} = \frac{\partial C}{\partial a}·a(1-a) $$

由前面可知 $\frac{\partial C}{\partial b} = (a-y)\sigma'(z)$ ，为了使激活函数梯度不包含 $\sigma'(z)$，可令：

$$ \frac{\partial a}{\partial z} = \sigma'(z) = 1 $$

连立两式可得微分方程：

$$ \frac{\partial C}{\partial a}·a(1-a) = (a-y) $$

采用分离变量法对两边进行不定积分：

$$ \int \partial C = \int \frac{a-y}{a(1-a)} \partial a = - \int ( \frac{y}{a} - \frac{1-y}{1-a} ) \partial a $$

可得：

$$ C = -[ y\ln a + (1-y)\ln (1-a) ] + const $$

# 结论
至此我们得出了用于替代的二次代价函数的式子：

$$ C = -\frac{1}{n} \sum_{x}{ [ y\ln a + (1-y)\ln (1-a) ] } $$

由于形式非常像[交叉熵](https://zh.wikipedia.org/wiki/%E4%BA%A4%E5%8F%89%E7%86%B5)，故取名**交叉熵代价函数**。