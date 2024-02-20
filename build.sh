if [ "$1" = "cv" ]; then
  docker image build -t ros2_pgr_dv ./.devcontainer --build-arg="OPENCV=true"
else
  docker image build -t ros2_pgr_dv ./.devcontainer
fi
