#! /bin/bash
brew install openssl libuv cmake zlib
git clone https://github.com/uWebSockets/uWebSockets
cd uWebSockets
git checkout e94b6e1
patch CMakeLists.txt < ../cmakepatch.txt
mkdir build
export PKG_CONFIG_PATH=/usr/local/opt/openssl/lib/pkgconfig
cd build
export OPENSSL_VERSION=1.0.2l
cmake -DOPENSSL_INCLUDE_DIR=/usr/local/Cellar/openssl/$OPENSSL_VERSION/include -DOPENSSL_CRYPTO_LIBRARY=/usr/local/Cellar/openssl/$OPENSSL_VERSION/lib/libcrypto.dylib -DOPENSSL_SSL_LIBRARY=/usr/local/Cellar/openssl/$OPENSSL_VERSION/lib/libssl.dylib ..
make
sudo make install
cd ..
cd ..
sudo rm -r uWebSockets
