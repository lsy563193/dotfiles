### 安装ssl

### 安装 git
sudo apt-get install git
git config --global user.email "you@example.com"
git config --global user.name "Your Name"

### 安装 Node.js
wget -qO- https://raw.github.com/creationix/nvm/master/install.sh | sh
export NVM_DIR="$HOME/.nvm"
[ -s "$NVM_DIR/nvm.sh" ] && \. "$NVM_DIR/nvm.sh"
nvm install stable
### 安装 Hexo
npm install -g hexo-cli
npm install hexo --save
###  tmux
sudo apt-get install tmux -y
### 设置shell为zshrc
sudo apt-get install zsh -y
chsh -s /bin/zsh
### 安装oh my zsh
wget https://github.com/robbyrussell/oh-my-zsh/raw/master/tools/install.sh -O - | sh
### 切换caps to ctrl
echo "setxkbmap -layout us -option ctrl:nocaps" >> ~/.zshrc
source ~/.zshrc
### 模糊搜索神器fzf
git clone --depth 1 https://github.com/junegunn/fzf.git ~/.fzf
~/.fzf/install
source ~/.zshrc
### autojump
git clone git://github.com/joelthelion/autojump.git ~/.autojump~
cd ~/.autojump~
./install.py
[[ -s /home/syue/.autojump/etc/profile.d/autojump.sh ]] && source /home/syue/.autojump/etc/profile.d/autojump.sh
autoload -U compinit && compinit -u

### install ros
sudo sh -c 'echo "deb http://packages.ros.org/ros/ubuntu $(lsb_release -sc) main" > /etc/apt/sources.list.d/ros-latest.list'
sudo apt-key adv --keyserver hkp://ha.pool.sks-keyservers.net:80 --recv-key 421C365BD9FF1F717815A3895523BAEEB01FA116
sudo apt-get update
sudo apt-get install ros-kinetic-desktop-full
apt-cache search ros-kinetic
sudo rosdep init
rosdep update
echo "source /opt/ros/kinetic/setup.zsh" >> ~/.zshrc
source ~/.zshrc
source /opt/ros/kinetic/setup.zsh