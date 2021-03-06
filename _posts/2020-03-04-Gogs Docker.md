---
layout: post
title:  Gogs Docker
date:   2020-03-04 22:23:01
comments: True
categories: Others
---

# 前期技术路线
需要搭建一个Git Server，用于中小型开发团队内部代码管理和版本维护，服务器为CentOS，内核为`3.10.0-862.el7.x86_64`，比较了多个git server，在易用性和适应场景的综合权衡之下选择了[Gogs](https://gogs.io)。

# 选择部署方案
刚开始采用源码编译安装的方式，下载了git、gogs源码进行编译安装，由于服务器系统的版本较低，在安装过程中缺少太多依赖库，内外网之间频繁导入折腾很久之后，思考需要采用一键式部署的方案。

考虑到系统的可移植性和后期需要热更新的需求，最终选择docker的部署方案。

## Gogs
由于内部服务器不能连接互联网，需将Gogs image pull之后save成gogs.tar，在拷入服务器进行load。
*这里建议Docker Hub可以学习Github的方式，在Releases tab页中添加一个版本发布的image.tar包供用户直接下载。*

## Database
在配置MySQL时踩了比较大的坑，当Gogs container部署首次启动以后，需要在web端配置对应的数据库。

先选择服务器中使用的MySQL，但是导入Gogs配置文件时，始终报一个字符超出764bytes的错误，最开始以为是username或者password中包含特殊字符导致编码过大无法存储，但是改成弱口令之后依然无法解决；

其次想到可能是MySQL数据库中默认的存储文件编码不是utf_8编码集导致的，修改MySQL存储文件编码之后依然无法解决；

最后求助StackOverflow：Gogs_image需要的MySQL最低版本要求为5.10，遂查看CentOS中MySQL版本为5.6。

于是数据库也采用docker_image的方式部署，这样也满足了维护的一致性。剩下的工作就是端口映射配置了。

# Commit rules
很多公司或者团队有建立自己本专业项目管理和技术知识分享的需求，在此基础上，制定了一套提交规范以供参考。

## 1. Code files
**代码文件提交时必须填写提交日志**（日志用祈使语气+现在时态填写）

* 分支暂存代码提交只添加`BT`日志，即：branch temp；
* 不要轻易在master分支上进行rebase操作；
* 关键词用tag标记；
* 提交代码后修改对应工单状态。

## 2. Binary files & Configurations
**非代码文件不与代码文件一同提交**

* 功能点实现时所有SDK一同单独提交；
* 版本完成发布时打标签提交。

# Idea share

1. 项目根目录创建git_img文件夹；
2. 完成新技术点时，截取相应效果图存放至git_img目录；
3. 在根目录的README.md文件中撰写相应的技术点说明；
4. 将git_img、README.md与新功能技术点完成代码文件一同commit。

团队内成员相互关注可在动态面板看到成员的动态信息，当有成果性README.md提交时能第一时间广播组内成员；当需要搜索某项功能技术点是否被前人实现时，可直接搜索动态面板内的信息，通过提交日志和关键字tag匹配能够直观地看到README内的成果效果说明和效果图，并且可以迅速定位到当次提交代码，方便参考。不同团队间引用并改进了功能技术点时可以通过pull request的方式进行补充完善。

同时，开发完成时可直接通过Repository首页上的README阅览整个项目所用到的新技术点，进而评估项目难度等级。

各团队可根据自己的团队规模和项目复杂度灵活使用，亦可提出改进工作流程相互交流:)