> **内容说明** : 文章内容翻译自ROS Wiki，也引用了部分《代码整洁之道》书中的内容。
> ROS C++代码规范与谷歌C++代码规范有诸多相似之处，本文主要讲述在编写ROS C++代码时需要遵守的编程规范。无论是ROS官方代码还是用户自定义代码，该规范都适用。<br>
> 感谢ROS wiki提供的资料，由于译者个人水平有限，文中难免有错误出现。 如有发现，请及时与我联系，感激不尽！！


<hr>


# 前言 什么是整洁代码


> 我喜欢优雅和高效的代码。代码逻辑应当直截了当，令缺陷难以隐藏；尽量减少依赖关系，使之便于维护；依据某种分层战略完善错误处理代码；性能调至最优，省得引诱别人做没规矩的优化，搞出一堆混乱来。整洁的代码只做好一件事。<br>
> —— **Bjarne Stroustrup**，C++语言发明者，《C++程序设计语言》（C++Programming Language）一书作者。

> 整洁的代码简单直接。整洁的代码如同优美的散文。整洁的代码从不隐藏设计者的意图，充满了干净利落的抽象和直截了当的控制语句。<br>
> —— **Grady Booch**，《面向对象分析与设计》（Object Oriented Analysis and Design with Applications）一书作者

> 整洁的代码应可由作者之外的开发者阅读和增补。它应当有单元测试和验收测试。它使用有意义的命名。它只提供一种而非多种做一件事的途径。它只有尽量少的依赖关系，而且要明确地定义和提供清晰的、尽量少的API。代码应通过其字面表达含义，因为不同的语言导致并非所有必需的信息均可通过代码自身清晰表达。<br>
> ——  **“老大” Dave Thomas**，OTl公司创始人，Eclipse战略“教父”

<hr>





# 1. 代码规范的重要性

代码风格很重要。 干净、一致的代码风格可以使代码更容易阅读、调试和维护。



 我们努力编写优雅的代码，不仅仅是为了简单地完成当下功能需求，还为了让这份代码持续存在，并在未来很多年内被其他开发人员重复使用、阅读和改进。**几名工程师合作开发一个项目时，做的最多的一件事情就是"看代码"， 每个人都需要能够看懂其他人的代码，这个时候，代码规范就显得尤为重要。** 

 为此，我们规定（并禁止）各种做法。我们致力于努力开发敏捷且合理的代码，其他人也可以很容易地理解这些代码。(Our goal is to encourage agile but reasoned development of code that can be easily understood by others.)

以下(文档)内容是参考准则，而非规则。(These are guidelines, not rules.) 除了极少数例外，本文档并未完全禁止任何特定的C++ 模式或功能(C++ pattern or feature)，而是描述了在大多数情况下运用的最佳实践。偏离此处给出的准则时，请务必仔细考虑您的选择，并在代码中记录您这么做的原因。

**最重要的是要保持一致性(consistent)。** 

- 在独立开发过程中尽可能遵循本指南。
- 如果您正在修改、编辑其他人编写的ROS package，请遵循该package中的现有样式约定。如您正在修改编辑的ROS Package遵循Google代码编程规范，那么您也要遵守Google代码编程规范（除非您要对整个程序包进行代码风格改版以遵循本指南）。


<hr>




# 2. ROS代码格式自动化工具

当我们致力于构建性能出色的机器人时，为什么要浪费您大量的宝贵开发时间来格式化代码呢？
这里介绍一款出色的工具—— clang-format，参考链接:<https://github.com/davetcoleman/roscpp_code_format>

在博客撰写时，clang_format工具已在2020年更新，并支持ROS Melodic系统。
在"使用方法"内容中，选择了`Linux命令行`以及`VS Code插件`来进行说明。更多其他工具请参考相关[readme.md文件](https://github.com/davetcoleman/roscpp_code_format/blob/master/README.md#usage)

## clang_format指南

> ### 2.1. 设置环境
>
> 1. 检索clang-format: `sudo apt-cache search clang-format`
> 2. **安装**clang_format: `sudo apt install clang-format-3.9`
> 3. **复制.clang-format文件**到机器人代码工程的根目录中，如: `~/catkin_ws/.clang-format`
> 4. 如果您有兴趣改进此配置文件，建议您**检查git repo和symlink**: `ln -s ~/roscpp_code_format/.clang-format ~/catkin_ws/.clang-format`
> 5. 现在，您的catkin_workspace工程文件夹中的任何文件都将使用此配置文件中所述的ROS编程规范进行格式化。



> ### 2.2. 运行clang_format
>
> #### 1. 命令行运行
>
> 1. 格式化单个文件：`clang-format-3.9 -i -style=file MY_ROS_NODE.cpp`
> 2. 递归格式化整个目录，包括子文件夹：`find . -name '*.h' -or -name '*.hpp' -or -name '*.cpp' | xargs clang-format-3.9 -i -style=file $1`
> 3. 可以将	`"格式化整个目录"`功能设置快捷指令`ros_format`,将下面代码添加到.bashrc或者.zshrc文件中。
>
> ```sh
>  alias ros_format="find . -name '*.h' -or -name '*.hpp' -or -name '*.cpp' | xargs clang-format-3.9 -i -style=file $1"
> ```
>
> 在终端中切换到catkin_ws工程根目录下，运行此命令`ros_format`，即可快速对代码格式化。
>
> #### 2. VS Code插件
>
> 1. 安装 [C/C++ 插件](https://marketplace.visualstudio.com/items?itemName=ms-vscode.cpptools).
>    ![在这里插入图片描述](https://img-blog.csdnimg.cn/2021021423550223.png)
> 2. 将[.clang-format文件](https://github.com/davetcoleman/roscpp_code_format/blob/master/.clang-format)添加到工程中，如: `~/catkin_ws/.clang-format`。
> 3. 打开首选项设置（ctrl + ,），搜索format，可勾选format on save 自动保存。
>    ![在这里插入图片描述](https://img-blog.csdnimg.cn/20210215000032567.png)
> 4. 在Seetings页面重新输入Clang_format_style，使得以下两个选项如图配置。
>    ![在这里插入图片描述](https://img-blog.csdnimg.cn/2021021500065213.png?x-oss-process=image,type_ZmFuZ3poZW5naGVpdGk,shadow_10,text_aHR0cHM6Ly9ibG9nLmNzZG4ubmV0L2N4czU1MzQ=,size_16,color_FFFFFF,t_70) 配置好后，在vscode中编辑代码，保存代码(`ctrl + s`)时编辑器会自动按照脚本规则检查和修改代码，使其满足ROS代码规范。
>
> **附: vscode插件工具推荐**
>
> 1. `Highlight Matching Tag` 突出显示匹配的开始或者结束标签
> 2. `Image Preview` 悬停时显示图像预览
> 3. `Indent Rainbow` 使文本的缩进着色，在每个步骤上交替使用四种不同的颜色。
> 4. `TODO Highlight` 在代码中突出显示TODO
> 5. `Better Comments` BetterComments可以帮助你编写便于阅读的注释。

<hr>




# 3. 如何对待不符合ROS编程规范的代码

在ROS代码规范发布之前，已经有许多ROS C++代码已经编写好了。因此，ROS代码库中有许多不符合ROS编程规范的代码。以下建议适合于使用不合规范代码的开发人员:

1. 所有新package均应符合本指南。
2. 除非您有足够的空闲时间，否则请勿进行转换现有代码库以符合本指南的规定。
3. 如果您是不合ROS编程规范代码包的**作者**，请尝试花时间更新代码以使其符合规范。
4. 如果您要对不合规范的package进行少量代码修改，**请遵循该程序包中的现有样式约定（如果有）。不要混合样式。**
5. 如果您要对不合格的包装进行重大工作，请趁此机会重新设置其样式以符合本指南。

<hr>




# 4. 良好的命名

以下例子表示ROS的命名体系：

| 范例 | 命名规范名称 | 规则 | 应用场景 | 示例
| --- | --- | --- | --- | --- | 
| **CamelCased**| 大驼峰(匈牙利命名法)|首字母大写，其后每个单词首字母大写 | 用于表示类名、类型。| `class ExampleClass;`(类名) <br> `class HokuyoURGLaser;`（带缩写单词的类名，缩写字母URG全大写）| 
|   **camelCased**| 小驼峰(匈牙利命名法)|首字母小写，之后单词首字母大写 | 方法、函数名 | `int exampleMethod(int example_arg);` 
|   **under_scored** | 小写+下划线 |名称仅使用小写字母，单词之间用下划线分隔。|   **ROS packages名称** ；<br>**Topics名**；<br> **Services名**；<br>**文件名**(.cpp、.c、.h)；<br>**库名**(注意格式是libxxx_yyy,而不是lib_xxx_yyy) ；<br> **命名空间**| `ros_openvino_toolkit` (**功能包名**)<br>  `action_server.h`(**文件名**)<br> `libmy_great_thing`(**库名**)  <br> `std::list<int> pid_list;` (**变量名**)<br>  `int example_int_;` (**成员变量以下划线`_`结尾**) <br> `int g_shutdown;` (**全局变量以`g_`开头**)
|   **ALL_CAPITALS**| 全部大写 |全部字母大写，单词之间用下划线分隔。 | 常量 | `PI`
| **__XXXX**| 前置下划线 | 前置下划线 `(__)`，在命名中不要使用前置下划线 | 系统保留| `__builtin_expect` (一般开发者不需要修改这方面内容)

<hr>




# 5. 许可证声明(License statements)

- 每个源文件和头文件必须在文件开头包含许可证和版权声明。
- 在ros-pkg和wg-ros-pkg存储库中，LICENSE目录包含许可证模板，以注释形式包含在C / C ++代码中。

文件开头加入版权公告，然后是文件内容描述。文件包含以下项:

1. 版权(Copyright statement): 如Copyright (c) 2018 Intel Corporation
2. 许可版本(License boilerplate): 为项目选择合适的许可证版本，如Apache2.0，BSD，LGPL，GPL
3. 作者(author line): 标识文件的原始作者

例: 

```cpp
/*
 * Copyright (c) 2018 Intel Corporation
 *
 * Licensed under the Apache License, Version 2.0 (the "License");
 * you may not use this file except in compliance with the License.
 * You may obtain a copy of the License at
 *
 *      http://www.apache.org/licenses/LICENSE-2.0
 *
 * Unless required by applicable law or agreed to in writing, software
 * distributed under the License is distributed on an "AS IS" BASIS,
 * WITHOUT WARRANTIES OR CONDITIONS OF ANY KIND, either express or implied.
 * See the License for the specific language governing permissions and
 * limitations under the License.
 */
```

如果你对其他人创建的文件做了重大修改，将你的信息添加到作者信息中，这样后续开发者有疑问时知道该联系谁。

<hr>




# 6. 代码风格

## 6.1.  编辑器自动格式化

1. 编辑器应处理大多数格式化任务，相关链接可以参考<http://wiki.ros.org/EditorHelp>，以及博客第2小节-自动格式化工具。
2. 以笔者最常用的编辑器-vim的配置文件为例，设置编辑器的配置文件

```yaml
" 自动缩进
set autoindent
set cindent
" Tab键的宽度
set tabstop=2
" 统一缩进为2
set softtabstop=2
set shiftwidth=2
" 使用空格代替制表符
set expandtab
" 在行和段开始处使用制表符
set smarttab
" 显示行号
set number
" 历史记录数
set history=1000
"搜索逐字符高亮
set hlsearch
set incsearch
"语言设置
set langmenu=zh_CN.UTF-8
set helplang=cn
" 总是显示状态行
set cmdheight=2
" 侦测文件类型
filetype on
" 载入文件类型插件
filetype plugin on
" 为特定文件类型载入相关缩进文件
filetype indent on
" 保存全局变量
set viminfo+=!
" 带有如下符号的单词不要被换行分割
set iskeyword+=_,$,@,%,#,-
```

## 6.2. 代码风格规范

- 每个块缩进2个空格。切勿插入tabs，设定编辑器将tab转为空格，UNIX / Linux下无条件使用空格。
- 命名空间的内容不缩进。
- 括号，无论是左右括号，都独占一列。

例：

 ```c 
 if(a < b)
{
  // do stuff
}
else
{
  // do other stuff
}
 ```

  - 每行最长120个字符
  - 每个头文件开头都应该包含#ifndef，防止重复包含。例:

```c
#ifndef PACKAGE_PATH_FILE_H
#define PACKAGE_PATH_FILE_H ...
#endif  
```

- 尽量不使用非ASCII字符，使用时必须使用UTF-8格式。

<hr>




# 7. 文档

代码必须有文档。 没有文档的代码即使现在可以运行，以后也很难维护。清晰、方便理解的注释不仅对阅读代码的人有好处，对自己也非常有用。开发人员经常会遇到这种情况：过一段时间之后，阅读自己的代码都有困难。而编写描述性的注释对于自己和团队都有好处。

我们将在project中使用[doxygen](http://www.doxygen.org/)工具自动生成文档。 Doxygen工具将解析您的代码，从特殊格式的块注释中提取分析出文档(函数、变量、类等旁边)，Doxygen也可用于构建更具描述性的自由格式文档。
![在这里插入图片描述](https://img-blog.csdnimg.cn/2021030123492772.png?x-oss-process=image,type_ZmFuZ3poZW5naGVpdGk,shadow_10,text_aHR0cHM6Ly9ibG9nLmNzZG4ubmV0L2N4czU1MzQ=,size_16,color_FFFFFF,t_70)
Doxygen的安装:

```cpp
sudo apt-get install doxygen
```

有关doxygen样式的注释示例，可以参考[rosdoc](http://wiki.ros.org/rosdoc)页面。
所有函数，方法，类，类变量，枚举和常量都应记录在案。

举个例子：在ros_openvino_toolkit中，param_manager.h中的部分代码:

```cpp
  /**
   * @brief Parse the give YAML file and generate parameters in ParamManager
   * instance
   * @param[in] path The absolute path of the YAML file which is to be parsed.
   * @return None.
   */
  void parse(std::string path);
```

这些注释包含了一些奇怪的内容，如“@return”、“@brief”、“@param”等， 这些并不是 C 语言的注释要求，而是Doxygen注释风格，使用其同名的软件，可以根据这种注释风格的代码自动生成 API 文档。

例如，在`ros_openvino_toolkit dev-2020.3`版本中，dynamic_vino_lib文件夹下已经有Doxyfile文件，此时在此文件夹下运行以下命令:

```shell
doxygen ./Doxyfile 
```

之后在工程中发现其生成了html和latex两个文件夹。分别对应latex和html风格的文档，读者可依据自己的喜好进行选择。
![在这里插入图片描述](https://img-blog.csdnimg.cn/20210302001939384.png?x-oss-process=image,type_ZmFuZ3poZW5naGVpdGk,shadow_10,text_aHR0cHM6Ly9ibG9nLmNzZG4ubmV0L2N4czU1MzQ=,size_16,color_FFFFFF,t_70)
译者在此以html文件夹为例，进入文件夹，打开index.html,即可查看整个工程的文档
![在这里插入图片描述](https://img-blog.csdnimg.cn/20210302002350154.png?x-oss-process=image/watermark,type_ZmFuZ3poZW5naGVpdGk,shadow_10,text_aHR0cHM6Ly9ibG9nLmNzZG4ubmV0L2N4czU1MzQ=,size_16,color_FFFFFF,t_70)
此时查看对象结构也是一目了然:
![在这里插入图片描述](https://img-blog.csdnimg.cn/20210302002921654.png?x-oss-process=image/watermark,type_ZmFuZ3poZW5naGVpdGk,shadow_10,text_aHR0cHM6Ly9ibG9nLmNzZG4ubmV0L2N4czU1MzQ=,size_16,color_FFFFFF,t_70)


更多关于Doxygen使用说明，推荐一个公开的pdf文档: [Doxygen Quick Reference](https://www.mitk.org/images/1/1c/BugSquashingSeminars$2013-07-17-DoxyReference.pdf), 读者可以自行参阅其中内容，笔者认为这个文档写的比较规范与全面。

更多关于Doxygen入门，可以参阅[Doxygen官网教程: Getting started](https://www.doxygen.nl/manual/starting.html)

实际上，无论是 pdf 还是网页版的 API 参考手册，它们的说明来源都是程序注释，所以在更多的时候，我们都是直接查看工程中源代码注释来了解如何使用。

<hr>




# 8. 控制台输出

避免使用C或者C++语言风格的字符串输出(比如`printf`， `cout`...)
可以使用rosconsole来满足您所有的输出需求，它提供了带有printf和stream-style的宏参数。不过其与printf不同的地方是：

1. 带颜色的控制台格式化输出
2. 详细的信息级别以及配置文件控制
3. 输出到`/rosout`话题上，可以在同一网络下的所有用户查看到。
4. 可以选择记录在磁盘上。

<hr>




# 9. 宏定义

尽可能避免使用宏。与内联函数和const变量不同，宏既没有类型也没有范围。

推荐参阅[谷歌cpp代码规范中对于宏的描述](https://google.github.io/styleguide/cppguide.html#Preprocessor_Macros)

<hr>




# 10. 预处理命令（#if与#ifdef）

对于条件编译（上面6.2小节解释的#ifndef头文件保护除外），请始终使用#if，而不是#ifdef。
有人可能会编写如下代码：

```cpp
#ifdef DEBUG
        temporary_debugger_break();
#endif
```

其他人可能会在关闭调试信息的情况下来编译代码，例如：

```cpp
cc -c lurker.cpp -DDEBUG = 0
```

这时候就有风险。

如果必须使用预处理器，请始终使用#if。即使根本没有定义DEBUG，它也可以正常工作，并且做正确的事情。

```cpp
#if DEBUG
        temporary_debugger_break();
#endif
```

<hr>




# 11. 输出参数

方法/函数的输出参数（例如：函数可以修改的变量），是通过指针而不是通过引用传递的。
例如：

```cpp
int exampleMethod（FooThing输入，BarThing *输出）;
```

相比之下，当通过引用传递输出参数时，调用者（或后续维护人员）被告知参数是否可以在不读取方法原型的情况下被修改

推荐参阅[Reference Arguments](<https://google.github.io/styleguide/cppguide.html#Reference_Arguments>)

<hr>




# 12. 命名空间

推荐使用namespace来限定代码范围，根据package的名称来选择一个描述性强的名称

切勿在头文件中使用`using`。这样做会污染包括头文件的所有代码的namespace。

在源文件(cpp)中使用`using`指令是可以接受的。但是最好使用`using-declarations`，它仅提取您打算使用的内容。
例如:

```cpp
using namespace std; // Bad, because it imports all names from std::
```

可以改为:

```cpp
using std::list;  // I want to refer to std::list as list
using std::vector;  // I want to refer to std::vector as vector
```

<hr>




# 13. 继承

使用组合通常比使用继承更适宜(这一点在GOF在《Design Patterns》里是反复强调的)。如果使用继承的话，只是用公共继承。
当子类继承父类时，子类包含了父基类所有数据以及操作的定义。
在C++实践中，继承主要用于两种场合: 实现继承和接口继承。

- 实现继承 (implementation inheitance)，子类继承父类的实现代码。
- 接口继承(interface inheritance)，子类仅继承父类的方法名称。

继承是定义和实现公共接口的合适手段。基类定义接口，子类实现该接口。(Inheritance is the appropriate way to define and implement a common interface. The base class defines the interface, and the subclasses implement it.)

继承还可以用于提供从基类到子类的通用代码。这种情况下不鼓励使用继承。(Inheritance can also be used to provide common code from a base class to subclasses. This use of inheritance is discouraged. )

在大多数情况下，“子类”可以包含“基类”的实例，并以较少的混淆可能性实现相同的结果。(discouraged. In most cases, the "subclass" could instead contain an instance of the "base class" and achieve the same result with less potential for confusion.)

子类重载虚拟(virtual)方法时，始终将其声明为`virtual`方法，以便读者了解正在发生的事情。(When overriding a virtual method in a subclass, always declare it to be virtual, so that the reader knows what's going on.)

强烈建议不要多重继承，多重继承允许子类拥有多个父类，它会引起无法容忍的混乱。

参考

1. [Google:Inheritance](https://google.github.io/styleguide/cppguide.html#Inheritance)
2. [Google:Multiple Inheritance](https://google.github.io/styleguide/cppguide.html#Multiple_Inheritance)

<hr>




# 14. 异常处理

与返回整数error codes相反，异常(Exceptions)是首选的错误报告机制。在测试框架中，异常确实十分好用。
对于现有代码，引入异常会牵连到所有依赖代码，异常会导致程序控制流无法通过查看代码确定——函数有可能在不确定的地方返回。所以有以下需要注意的地方:

- 始终在每个相关函数/方法上，记录您的package可能会抛出哪些异常。
- 不要抛出析构函数的异常。
- 不要从您不直接调用的回调中引发异常。
- 如果您在package中选择使用错误代码代替异常，则仅使用错误代码。 始终如一。

## 14.1 编写抛出异常时安全的代码

当您的代码可以被异常中断时，您必须确保当堆栈溢出时，相关资源将被释放。特别是，必须释放互斥锁，并且必须释放堆分配的内存。

更多内容可以参考[StackOverflow : do you really write exception safe code?](<https://stackoverflow.com/questions/1853243/do-you-really-write-exception-safe-code>)

<hr>




# 15 枚举

命名您的枚举，例如

```cpp
namespace Choices
{
  enum Choice
  {
     Choice1,
     Choice2,
     Choice3
  };
}
typedef Choices::Choice Choice;
```

这样可以防止枚举污染它们所在的命名空间。
枚举中的单独的item引用：Choices :: Choice1。
typedef仍然允许声明Choice enum而不是命名空间。

如果您使用的是C ++ 11和更高版本，则可以使用范围枚举。例如

```cpp
enum class Choise
{
    Choice1,
    Choice2,
    Choice3
};
Choise c = Choise::Choice1;
```

[Enumerator Names](https://google.github.io/styleguide/cppguide.html#Enumerator_Names)

<hr>




# 16. 全局变量

不建议使用全局变量（无论变量还是函数）。它们会污染namespace，并使代码的可重用性降低，耦合性大大提高，使得维护变得困难。它们阻止代码的多个实例化，并使多线程编程成为一场噩梦。(They prevent multiple instantiations of a piece of code and make multi-threaded programming a nightmare.)

大多数变量和函数应在类内部声明。其余应在namespace中声明。

例外：文件可能包含main()函数和一些全局的小辅助函数。但是请记住，有一天这些辅助功能可能对其他人有用。

**参考阅读**

- [Google:Static and Global Variables](https://google.github.io/styleguide/cppguide.html#Static_and_Global_Variables)
- [Google:Nonmember, Static Member, and Global Functions](https://google.github.io/styleguide/cppguide.html#Nonmember,_Static_Member,_and_Global_Functions)

<hr>





# 17. Static class variables

不建议使用静态类变量。它们阻止代码的多个实例化，并使多线程编程成为一场噩梦。



<hr>




# 18. 调用exit()

仅在应用程序中定义明确的退出点(exit point)时调用exit()。
切勿在库中调用exit()。

<hr>




# 19. 断言

使用断言检查`先决条件`，`数据结构完整性`和`内存分配器的返回值`。
断言比编写条件语句要好，后者很少会被执行。

不要直接调用assert（）。而是使用在ros / assert.h中声明的以下函数之一（rosconsole软件包的一部分）：

-  ROS_ASSERT(x > y);
-  ROS_ASSERT_MSG(x > 0, "Uh oh, x went negative.  Value = %d", x);
-  ROS_ASSERT_CMD(x > 0, handleError(...));
-  ROS_BREADK();

```cpp
/** ROS_ASSERT asserts that the provided expression evaluates to
 * true.  If it is false, program execution will abort, with an informative
 * statement about which assertion failed, in what file.  Use ROS_ASSERT
 * instead of assert() itself.
 * Example usage:
 */
   ROS_ASSERT(x > y);
```

```cpp
/** ROS_ASSERT_MSG(cond, "format string", ...) asserts that the provided
 * condition evaluates to true.
 * If it is false, program execution will abort, with an informative
 * statement about which assertion failed, in what file, and it will print out
 * a printf-style message you define.  Example usage:
 */
   ROS_ASSERT_MSG(x > 0, "Uh oh, x went negative.  Value = %d", x);
```

```cpp
/** ROS_ASSERT_CMD(cond, function())
 * Runs a function if the condition is false. Usage example:
 */
   ROS_ASSERT_CMD(x > 0, handleError(...));
```

```cpp
/** ROS_BREAK aborts program execution, with an informative
 * statement about which assertion failed, in what file. Use ROS_BREAK
 * instead of calling assert(0) or ROS_ASSERT(0). You can step over the assert
 * in a debugger.
 * Example usage:
 */
   ROS_BREADK();
```

不要在断言中做任何工作；仅检查逻辑表达式。取决于编译环境的设置，可能不会执行该断言。
通常会开发启用了断言检查的软件，以捕获异常情况(in order to catch violations)。
当软件即将完成时，并且在进行大量测试时发现断言始终是正确的时，您将使用一个标志从编译中删除断言，从而使它们不占用任何空间或时间。
catkin_make的以下选项将为所有ROS package定义`NDEBUG`宏，从而删除断言检查。

```make
catkin_make -DCMAKE_CXX_FLAGS:STRING="-DNDEBUG"
```

**注意**：当您使用此命令运行cmake时，它将重新全部编译，并且在后续运行catkin_make时会记住相关设置，直到删除build和devel目录重新编译为止。


<hr>




# 20. 测试

参考阅读: [GTEST](http://wiki.ros.org/gtest)

<hr>




# 21. 可移植性

保持C ++代码的可移植性很重要。以下是注意事项：

1. 不要将uint用作类型。而是使用unsigned int。
2. 从std命名空间中调用isnan()，即`std :: isnan()`

<hr>




# 22. 弃用 Deprecation

1. 当要弃用package中的头文件时，可以包含相关警告：

```c
#warning mypkg/my_header.h has been deprecated
```

2. 当要弃用一个函数时，请添加不建议使用的描述：

```c
ROS_DEPRECATED int myFunc();
```

3. 当要弃用一个类时，请弃用其构造函数和所有静态函数：

```cpp
class MyClass
{
public:
  ROS_DEPRECATED MyClass();

  ROS_DEPRECATED static int myStaticFunc(); 
};
```

<hr>


##  背景资料：糟糕的代码与混乱的代价 (节选自《代码整洁之道》) 

> 只要你干过两三年编程，就有可能曾被某人的糟糕的代码绊倒过。如果你编程不止两三年，也有可能被这种代码拖过后腿，进度延缓的情况会很严重。有些团队在项目初期进展迅速，但有那么一两年的时间却慢如蜗行。对代码的每次修改都影响到其他两三处代码，修改无小事。每次添加或修改代码，都得对那堆扭纹柴了然于心，这样才能往上扔更多的扭纹柴。这团乱麻越来越大，再也无法理清，最后束手无策。<br>
> 随着混乱的增加，团队生产力也持续下降，以致趋向于零。当生产力下降时，管理层就只有一件事可做了：增加更多人手到项目中，期望提升生产力。可是新人并不熟悉系统的设计。他们搞不清楚什么样的修改符合设计意图，什么样的修改违背设计意图。而且，他们以及团队中的其他人都背负着提升生产力的可怕压力。于是，他们只会制造更多的混乱，驱动生产力向零那端不断下降。
> ![在这里插入图片描述](https://img-blog.csdnimg.cn/2021020412534321.png?x-oss-process=image,type_ZmFuZ3poZW5naGVpdGk,shadow_10,text_aHR0cHM6Ly9ibG9nLmNzZG4ubmV0L2N4czU1MzQ=,size_16,color_FFFFFF,t_70)

# 参考资料

## 参考链接

1. ROS Cpp StyleGuide: <http://wiki.ros.org/CppStyleGuide#Macros>
2. Google C++代码规范
   - 英文版:  <https://google.github.io/styleguide/cppguide.html#Background>
   - 中文版:  <https://zh-google-styleguide.readthedocs.io/en/latest/google-cpp-styleguide/>
3. VS Code C++ 代码格式化方法(clang-format)： <https://blog.csdn.net/core571/article/details/82867932>
   <https://blog.csdn.net/softimite_zifeng/article/details/78357898>
4. 关于VSCode插件: <https://mp.weixin.qq.com/s/CgMppoLoHJ5kpZcUIuJnNg>

## 参考文献

1. [代码整洁之道 [美] 罗伯特·C.马丁 (Robert C.Martin)著](https://www.epubit.com/bookDetails?id=UBb601dc20509c)
2. [C++代码整洁之道 C++17可持续软件开发模式实践 [德] 斯提芬·罗特（StephanRoth） 著](https://book.douban.com/subject/33442738/)
3. [Effective Debugging [希]迪欧米迪斯·斯宾奈里斯(Diomidis Spinellis)](https://book.douban.com/subject/27064532/)

<hr> 