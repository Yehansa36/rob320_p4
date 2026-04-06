echo "Installing Eigen 3.4.0 ..."
wget https://gitlab.com/libeigen/eigen/-/archive/3.4.0/eigen-3.4.0.tar.gz
tar -xvf eigen-3.4.0.tar.gz
cd eigen-3.4.0
mkdir build
cd build
cmake ..
sudo make install -j4
cd ../..
rm -rf eigen-3.4.0 eigen-3.4.0.tar.gz

echo "Installing nlohmann/json 3.11.3 ..."
wget https://github.com/nlohmann/json/archive/refs/tags/v3.11.3.tar.gz
tar -xvf v3.11.3.tar.gz
cd json-3.11.3
mkdir build
cd build
cmake ..
sudo make install -j4
cd ../..
rm -rf json-3.11.3 v3.11.3.tar.gz

echo "Unzipping robot models ..."
mkdir -p ~/.rix/models/
unzip robots/models.zip
cp -r models/* ~/.rix/models/
rm -rf models
