# syntax=docker/dockerfile:1

# This file specifies the dev container used for LPL flight software development. It includes all dependencies
# required to build, test, and flash.

FROM ubuntu:noble

SHELL ["/bin/bash", "-o", "pipefail", "-c"]
ENV TZ="America/Los_Angeles" LC_ALL=en_US.UTF-8 LANG=en_US.UTF-8

# Sensible utilities that every system should probably have
RUN << EOF
apt update
apt -y upgrade
apt -y install sudo nano git curl wget jq yq iproute2 netcat-openbsd gh locales
EOF

# Zephyr dependencies -- from https://docs.zephyrproject.org/latest/develop/getting_started/index.html, but we
RUN << EOF
apt -y install --no-install-recommends git cmake ninja-build gperf \
    ccache dfu-util device-tree-compiler python3-dev python3-venv python3-tk \
    xz-utils file make gcc gcc-multilib g++-multilib libsdl2-dev libmagic1
EOF
ENV ZEPHYR_TOOLCHAIN_VARIANT=zephyr

# C/C++ linters, static analysis
RUN << EOF
apt -y install cppcheck clang-tidy
EOF

# Labjack library installation
RUN << EOF
apt -y install unzip
mkdir -p /tmp/labjack-install
curl -o /tmp/labjack-install/installer.zip https://files.labjack.com/installers/LJM/Linux/x64/release/LabJack-LJM_2025-05-07.zip
unzip /tmp/labjack-install/installer.zip -d /tmp/labjack-install
/tmp/labjack-install/labjack_ljm_installer.run
rm -rf /tmp/labjack-install
EOF

# Other dev dependencies
RUN << EOF
apt -y install protobuf-compiler build-essential libboost-all-dev
EOF

# Create non-root user
RUN << EOF
useradd --create-home --shell /bin/bash --gid 1000 --groups sudo lpl
echo lpl ALL=\(root\) NOPASSWD:ALL > /etc/sudoers.d/lpl
chmod 0440 /etc/sudoers.d/lpl
EOF
USER lpl

# Python
RUN << EOF
curl -LsSf https://astral.sh/uv/install.sh | sh
. "$HOME/.local/bin/env"
cd $HOME && uv venv
uv tool install ruff@latest
uv pip install west
EOF

# Install zephyr dependencies that require West -- namely, toolchain and SDK
COPY --chmod=777 west.yml /home/lpl/arty/west.yml
RUN << EOF
. "$HOME/.local/bin/env"
. /home/lpl/.venv/bin/activate
west init -l /home/lpl/arty
cd /home/lpl || exit 1
west config manifest.project-filter -- +nanopb
west update
uv pip install -r /home/lpl/zephyr/scripts/requirements.txt
uv pip install protobuf grpcio-tools
west zephyr-export
west sdk install -t arm-zephyr-eabi
sudo rm -rf /home/lpl/arty
EOF

# Rust
RUN << EOF
curl --proto '=https' --tlsv1.2 -sSf https://sh.rustup.rs | sh -s -- -y
. "$HOME/.cargo/env"
rustup component add rustfmt
EOF

# Build flasherd-client and store it at ~/prebaked-bin so it can be copied into ~/arty/bin upon dev container startup.
COPY Cargo.toml /home/lpl/arty/Cargo.toml
COPY Cargo.lock /home/lpl/arty/Cargo.lock
COPY flasherd /home/lpl/arty/flasherd
COPY flasherd-client /home/lpl/arty/flasherd-client
COPY api /home/lpl/arty/api
RUN << EOF
sudo chown -R lpl ~/arty
sudo chmod -R 0777 ~/arty

. ~/.cargo/env
cd ~/arty || exit 1
cargo build --package flasherd-client --release

mkdir -p ~/prebaked-bin
mv ~/arty/target/release/flasherd-client ~/prebaked-bin/flasherd-client

sudo rm -rf ~/arty
EOF

# Cosmetic changes
RUN << EOF
# Remove sudo tutorial from startup
touch ~/.sudo_as_admin_successful

# Oh-my-bash terminal theme
bash -c "$(curl -fsSL https://raw.githubusercontent.com/ohmybash/oh-my-bash/a59433af4d5c68861d66b2b9dcf8ada7f1b0e6f5/tools/install.sh)" --unattended
mkdir -p ~/.oh-my-bash/themes/lpl-term
cat << "END" > ~/.oh-my-bash/themes/lpl-term/lpl-term.theme.sh
#! bash oh-my-bash.module

# Derivative of https://github.com/ohmyzsh/ohmyzsh/blob/master/themes/tonotdo.zsh-theme, but with a delta as the terminator.

SCM_THEME_PROMPT_PREFIX=" ${_omb_prompt_purple}"
SCM_THEME_PROMPT_SUFFIX=" ${_omb_prompt_normal}"
SCM_THEME_PROMPT_DIRTY=" ${_omb_prompt_brown}✗"
SCM_THEME_PROMPT_CLEAN=" ${_omb_prompt_green}✓"
SCM_GIT_SHOW_DETAILS="false"

function _omb_theme_PROMPT_COMMAND() {
    PS1="${_omb_prompt_olive}\u${_omb_prompt_normal}${_omb_prompt_teal}@\h${_omb_prompt_normal}${_omb_prompt_purple} ${_omb_prompt_normal}${_omb_prompt_green}\w${_omb_prompt_normal}${_omb_prompt_red} Δ${_omb_prompt_normal} "
}

_omb_util_add_prompt_command _omb_theme_PROMPT_COMMAND
END
sed -i 's/^OSH_THEME=.*$/OSH_THEME="lpl-term"/' ~/.bashrc

# LPL logo
cat << "END" >> ~/.bashrc
lplred="\e[31;1m"
lplylw="\e[33;1m"
lplbld="\e[39;1m"
style_rst="\e[0m"
echo -e "   ${lplred}__   ${lplylw}___ ${lplred} __
${lplred}  / /  ${lplylw}/ _ \\\\${lplred}/ /
${lplred} / /__${lplylw}/ ___${lplred}/ /__
${lplred}/____${lplylw}/_/  ${lplred}/____/
${lplbld}Welcome!${style_rst}"

# Ensure path is correct for flasherd test
export PATH="$PATH:/home/lpl/arty/bin"

# Show flasherd status
if $HOME/arty/scripts/flasherd-connection-test.sh > /dev/null 2>&1; then
    grnbld="\e[32;1m"
    echo -e "flasherd is ${grnbld}active${style_rst}".
else
    redbld="\e[31;1m"
    echo -e "flasherd is ${redbld}inactive${style_rst}".
fi
END
EOF

# Final environment setup
RUN << EOF
# Activate venv
echo '. "$HOME/.venv/bin/activate"' >> "$HOME/.bashrc"
# Populate zephyr vars
echo '. $HOME/zephyr/zephyr-env.sh' >> "$HOME/.bashrc"
# Set initial working directory
echo 'cd /home/lpl/arty' >> "$HOME/.bashrc"
# Ensure that workspace is put in correct location
sudo ln -s /workspaces/arty /home/lpl/arty
EOF
ENV PYTHONPATH="/home/lpl/zephyr/scripts/west_commands" PATH="$PATH:/home/lpl/arty/bin" ZEPHYR_TOOLCHAIN_VARIANT=zephyr
