# 在Ubuntu中使用Zsh

> Zsh与bash都是shell的一种，zsh的扩展性和可玩性更高，在大多数场景下兼容bash


## 一系列安装操作

### 安装Zsh

```shell
sudo apt-get install zsh
```

将终端的默认shell切换为zsh

```shell
chsh -s $(which zsh)
```

### 安装Oh-my-zsh

```shell
sh -c "$(wget https://raw.github.com/ohmyzsh/ohmyzsh/master/tools/install.sh -O -)"
```

### 安装Hack nerd font

```shell
git clone https://github.com/ryanoasis/nerd-fonts.git --depth 1
cd nerd-fonts
./install.sh
```

然后在Terminator中选择Hack nerd font

### 安装p9k主题

```shell
sudo apt install zsh-theme-powerlevel9k
```

配置环境变量

```shell
echo "source /usr/share/powerlevel9k/powerlevel9k.zsh-theme" >> ~/.zshrc
```

## 个性化配置操作

懒得折腾可以直接在p9k的[github wiki](https://github.com/Powerlevel9k/powerlevel9k/wiki/Show-Off-Your-Config)中copy别人的.zshrc

安装其他插件可以实现更多强大的功能（如自动补全）

![](../imgs/terminal.png)



## 解决使用rosmon找不到包并出现报错问题

> [!NOTE]
>
> 使用`rosmon`启动launch文件的时候按tab会找不到包，这是`rosmon`在zsh中的一个bug，见Github中rosmon的[*issue*](https://github.com/xqms/rosmon/issues/92)

**解决办法**：在工作空间`source`完后再

```shell
➜  source /opt/ros/noetic/etc/catkin/profile.d/50-rosmon.zsh
```

或者在.zshrc中创建别名

```shell
alias sou='source ./devel/setup.zsh && source /opt/ros/noetic/etc/catkin/profile.d/50-rosmon.zsh'
```

