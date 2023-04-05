# UGAS编译前准备

## 确保UGAS处于Release x64模式

在一切工作开始前，请打开UGAS，检查是否处于如图编译模式。

![](images/de309db8-e726-46c6-8639-e689851c7f9e.png?raw=true)

由于Debug模式会带来许多性能问题（帧数降至个位数），UGAS一般在Release模式下运行。但若一定要使用Debug模式进行局部动态调试，请另行配置，配置方式在本文最后一段。

<br/>

## 配置OpenCV

**Step1 下载并安装OpenCV**

访问 [Releases - OpenCV](https://opencv.org/releases/) 找到最新版的OpenCV（在最上方且标有*号），点击“Windows”按钮下载。

下载完成后，将得到的zip文件就地解压，双击里面的distrib.7z.exe，会出现界面Extract to（解压至），如图将下面的文本框中的内容修改为“D:\”，指明OpenCV的安装路径为D盘根目录。

![](images/136cf546-0a1f-4c4d-8365-359a4699e60f.png?raw=true)

注意：不要使用其他安装目录，这会导致你与团队中的其他成员无法同步。如果你没有D盘，可以使用Windows自带的虚拟分区功能，将文件夹虚拟为分区（自行查阅资料）。

**Step2 配置环境变量**

打开 D:\opencv\build\x64 文件夹，内装有命名风格类似“vc**”的文件夹

按下Windows键，输入env，点击“编辑系统环境变量”

![](images/158523ee-7503-4f71-a918-dab6a2062171.png?raw=true)

在弹出的对话框中，点击“环境变量(N)”，会弹出一个新对话框，选择“Path”，点击“编辑”

![](images/7f6caa9d-3fcd-49a6-b3e7-6c9b1cc72cf4.png?raw=true)

如图所示，点击“新建”，把刚才的文件夹按照 D:\opencv\build\x64\vc**\bin 的格式添加进去。

注意：如果有一个文件夹就添加一项，如果有两个文件夹就添加数字大的那一项，不要遗漏后面的 "\bin"。

![](images/be22d535-60a8-4eb1-862d-d0b2c61f5c7b.png?raw=true)

**Step3 配置openCV.props**

这里要分两种情况讨论，请先打开本文件同目录下的openCV.props文件，在第五行有注释：文件适用于OpenCV \*.\*.\*。

情况1：该文件与你安装的OpenCV版本一致，那么非常简单，你只需要将文件拷贝到 D:\openCV\openCV.props 处，即可完成配置。

情况2：该文件与你安装的OpenCV版本不一致（版本过旧），那么你需要对此文件做如下修改。

1. 将\<LibraryPath\>中 vc** 的数字改为你刚才添加进环境变量中的那个数字
   ![](images/ac1d4a74-0807-44d1-a123-3e593f709e06.png?raw=true)
2. 打开刚才LibraryPath中的文件夹 D:\\opencv\\build\\x64\\vc\*\*\\lib，找到里面命名风格类似opencv_world\*\*\*.lib的文件，把\<AdditionalDependencies\>中opencv_world\*\*\*.lib的数字改为这个文件的数字（注意不要有d）。
   ![](images/5ca41cc8-22f5-4580-9063-abc30d9dce88.png?raw=true)
3. 保存修改，把文件拷贝到 D:\openCV\openCV.props 处，完成配置。
4. 最好把你修改此文件后的UGAS推送到Github云端，造福大众。

完成后的opencv目录如图所示。

![](images/83d0fbf8-9431-4c3d-a1e8-b34dfea2af35.png?raw=true)

**Step4 重启电脑**

修改环境变量后，一般要重启才能生效，请重启你的电脑。

<br/>

## 配置视频源

由于Github上传文件大小限制，调试用视频源没有被上传，需要手动配置。

![](images/57105afa-8617-4c12-a3ca-c8fcf633ed25.png?raw=true)

在UGAS.sln的同级目录下，新建一个文件名为 resources 的文件夹。

在视觉群中找到 resources.zip 文件，下载并解压至该文件夹中。注意检查不要解压出嵌套目录，视频文件应直接放在resources目录下。

![](images/4b0e1ff6-fee1-4933-98a7-85df794bd102.png?raw=true)

<br/>

## 编译UGAS

完成以上所有步骤后，就可以进行UGAS的编译了。打开Visual Studio，点击“本地Windows调试器”，你会看到它愉快地跑起来了。

<br/>

## 配置Debug x64编译模式

尽管由于严重的性能问题，我们一般情况下不推荐使用Debug x64编译模式。但Release x64模式也有它的缺陷，它的动态调试功能完全缺失，断点几乎无法命中，当我们需要对代码进行局部动态调试时，我们需要Debug x64编译模式。

具体操作步骤如下：

**Step1 打开 D:\opencv文件夹**

**Step2 将 openCV.props 复制一份并重命名为 openCV.debug.props**

**Step3 修改 openCV.debug.props**

如图，将\<AdditionalDependencies\>中opencv_world\*\*\*后面加上一个小写的“d”。

![](images/9fad27fb-1caf-4441-a23b-c0e330aad9f5.png?raw=true)

保存后即可切换到Debug x64模式进行编译。
