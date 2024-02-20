cd ./.devcontainer
if [ "$1" = "cv" ]; then
  docker image build -t ros2_pgr_dv . --build-arg="OPENCV=true"
else
  docker image build -t ros2_pgr_dv .
fi
cd ..

