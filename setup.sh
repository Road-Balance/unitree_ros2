
# Install Boost

wget -O boost_1_78_0.tar.gz https://sourceforge.net/projects/boost/files/boost/1.78.0/boost_1_78_0.tar.gz/download
tar xzvf boost_1_78_0.tar.gz
rm -rf boost_1_78_0.tar.gz

cd boost_1_78_0

sudo ./bootstrap.sh
sudo ./b2 install

# Install lcm

wget -O lcm-1.4.0.tar.gz https://github.com/lcm-proj/lcm/archive/refs/tags/v1.4.0.tar.gz
tar xzvf lcm-1.4.0.tar.gz
rm -rf lcm-1.4.0.tar.gz

cd lcm-1.4.0/

mkdir build && cd build
cmake .. && make
sudo make install

# Install miscellaneous

sudo apt install libglib2.0-dev build-essential