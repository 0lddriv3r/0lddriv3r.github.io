---
layout: post
title:  QtGraphics绘图
date:   2018-01-24 16:27:46
comments: True
categories: Qt
---

最近工作需要，大量的Qt绘图，看到一篇不错的Qt绘图机制介绍：[吉米呦的博客](http://blog.sina.com.cn/s/blog_4a33cfca01015ppv.html)，在这里给大家分享。同时，Qt自带的很多Example也是很好的学习资料，基本能够满足你绝大多数需求。

Qt Graphics模块用于显示二维的图形图像，所以三维的事情就不要找它了，应该去找Qt的OpenGL模块。主要由三部分组成，分别是 **View**, **Scene**, **Item**。

QGraphicsView负责窗口显示，它继承自QWidget，因此是一个标准的Qt窗口类，Qt窗口类一般的操作QGraphicsView都支持。QGraphicsScene是一个视图，它不能够单独存在，必须关联到至少一个QGraphicsView。这两者的关系就和MVO架构中的文档和视图的关系类似，View是视图，负责显示；Scene是文档，负责存储数据。所以从这个角度出发，我们可以这样认为，一个Scene可以关联到多个View，就好比一份数据可以有多个视图去查看它一样。

Item则是具体要显示的东西。最基本的Qt类就是 **QGraphicsItem**，一般如果要显示自定义的形状通常的做法是继承自QGraphicsItem，然后去实现它的两个纯虚函数`boundingRect`和`paint`，它们的形式化申明如下：
```c++
virtual QRectF boundingRect () const = 0;
virtual void paint ( QPainter * painter, const QStyleOptionGraphicsItem * option, QWidget * widget = 0 ) = 0;
```
boundingRect就是返回该Item的包围盒，用于Graphics内部的碰撞检测以及选取等计算。paint则是用于Item的重绘。

除最基本的QGraphicsItem之外，Qt还定义了若干基本形状的Item，例如QGraphicsEllipseItem、QGraphicsLineItem、QGraphicsRectItem等等，具体可查Qt文档。如果想要绘制的Item形状和这些基本类型接近，不妨直接继承自这些类，然后在paint函数中稍加修改。

Qt之所以提供Graphics模块是为了对二维图形编程方面提供基本的常用的平台功能支持，例如拖动、选择、旋转、动画、缩放、碰撞检测等等。这些功能，简言之，如果我们要用，是不必像在MFC上那样大费周折的，Qt Graphics提供了很好的操作接口。例如，要使得Item能够被选择，只需要设置它的Flag：
`setFlag(QGraphicsItem::ItemIsMovable);`
即可。移动、是否能获取焦点等都类似。

Qt默认情况下，对于选中的Item，如果你是继承自特殊形状的子类，则会在Item的包围框上显示虚框。若要去掉该虚框，则在paint函数中加入如下代码：
```c++
QStyleOptionGraphicsItem op(*option ); 
op.state = QStyle::State_None;
```
然后调用父类的paint函数时，将op作为option传入即可。如果要使得选中的Item呈现不同的状态（颜色、大小等）也是在paint函数中加以修改。

QGraphicsView和QGraphicsScene以及QGraphicsItem都能够接受鼠标、键盘事件，那他们的关系又是如何呢？首先从消息流来看，先是View，然后是Scene，最后是Item。任何阶段若节流事件，则下一层就不会再接受到事件了。对于QGraphicsView和QGraphicsScene，我们应该在哪里处理鼠标或者键盘事件呢？其实两者都是可以的，具体看需求。联系之前所说的文档视图关系可以知道，若应用程序中只有一个View和一个Scene，那么其实在哪里实现都无所谓；当时如果一个Scene对应了多个View，则就有区别了。如果具体到一个视图的操作，应该在View类中实现；如果是Scene需要统一进行处理的，那么就应该在Scene中实现。

Scene更新的时候，尽可能将更新范围限制在最小内，毕竟update全局的话需要消耗较多的资源，尤其是显示内容比较大的时候，会不流畅。

获取鼠标下面的item方法是itemAt。若在View中调用此函数，注意它的参数坐标是View中的坐标，即不需要再做任何的转化。

**GraphicsScene中可以内嵌入标准的QWidget**，但是其行为和正常的widget略有不同。我发现的一点是：只有真正在Scene中显示的时候，其内部控件才是真正存在的，否则ui内的指针均无效。估计是为了节约资源考虑的吧。这点切记！

QGraphicsItemAnimation可以实现一些简单的动画操作，亲测效果还是不错的，较为平滑。