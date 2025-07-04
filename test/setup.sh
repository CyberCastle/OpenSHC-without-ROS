#!/usr/bin/env bash
set -e

if [[ "$(uname)" == "Darwin" ]]; then
  if brew ls --versions eigen >/dev/null 2>&1; then
    echo "Eigen library already installed via Homebrew"
  else
    echo "Installing Eigen library with Homebrew..."
    brew update
    brew install eigen
  fi
else
  if dpkg -s libeigen3-dev >/dev/null 2>&1; then
    echo "Eigen library already installed"
  else
    echo "Installing Eigen library via apt..."
    sudo apt-get update
    sudo apt-get install -y libeigen3-dev
  fi
fi
