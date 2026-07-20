# Microsoft GPG Key 등록
sudo apt update
sudo apt install wget gpg

wget -qO- https://packages.microsoft.com/keys/microsoft.asc \
| gpg --dearmor \
> packages.microsoft.gpg

sudo install -D -o root -g root -m 644 \
packages.microsoft.gpg \
/etc/apt/keyrings/packages.microsoft.gpg

# VS Code Repository 등록
echo "deb [arch=amd64 signed-by=/etc/apt/keyrings/packages.microsoft.gpg] https://packages.microsoft.com/repos/code stable main" \
| sudo tee /etc/apt/sources.list.d/vscode.list

# 패키지 목록 갱신
sudo apt update

# VS Code 설치
sudo apt install code
