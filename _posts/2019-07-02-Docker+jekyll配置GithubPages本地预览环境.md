---
layout: post
title:  Docker+jekyll配置GithubPages本地预览环境
date:   2019-07-02 19:30:13
comments: True
categories: Others
---

由于我的**Github Pages**依赖于jekyll，每次有较复杂的排版时总是会出现莫名其妙的问题（可能是jekyll解析的Markdown格式有出入），所以需要配置一个本地的预览环境。

但是不想主机OS上安装过多的定制化功能，本着**谁消费谁维护**的原则，同时Docker Hub提供了现成的镜像，技术路线选择了**Docker**容器。

# Docker jekyll
1. 首先pull jekyll解析环境的image：
```
docker pull jekyll/jekyll
```

2. 在GitHub Pages目录下启动jekyll容器并映射到宿主机4000端口：
```
docker run --mount type=bind,source=$(pwd),target=/srv/jekyll \
-p 4000:4000 --name blog -it jekyll/jekyll \
jekyll serve
```
输出日志如下：
```
ruby 2.6.3p62 (2019-04-16 revision 67580) [x86_64-linux-musl]
Configuration file: /srv/jekyll/_config.yml
       Deprecation: The 'gems' configuration option has been renamed to 'plugins'. Please update your config file accordingly.
            Source: /srv/jekyll
       Destination: /srv/jekyll/_site
 Incremental build: disabled. Enable with --incremental
      Generating... 
    Liquid Warning: Liquid syntax error (line 4): Unexpected character + in "post in site.categories.C++" in blog/C++/index.html
                    done in 9.279 seconds.
 Auto-regeneration: enabled for '/srv/jekyll'
    Server address: http://0.0.0.0:4000/
  Server running... press ctrl-c to stop.
```
根据提示就可以通过主机的`http://0.0.0.0:4000/`进行预览了。

3. 由于经常使用，所以在第2步中设置了container的别名blog，以后就可以通过容器别名启动了：
```
docker start -i blog
```