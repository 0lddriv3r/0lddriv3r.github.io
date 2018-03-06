# coding=utf-8

import os
import time
import fire

def create_blog(blog_name, category):
	date_time_str = time.strftime("%Y-%m-%d %H:%M:%S", time.localtime())
	date_str = date_time_str.split(' ')[0]
	file_name = './' + date_str + '-' + blog_name + '.md'
	with open(file_name, 'w') as f:
		text = '''---\nlayout: post\ntitle:  {blog_name}\ndate:   {date_time}\ncomments: True\ncategories: {category}\n---\n\n'''.format(blog_name=blog_name, date_time=date_time_str, category=category)
		f.write(text)


if __name__ == '__main__':
    fire.Fire(create_blog)