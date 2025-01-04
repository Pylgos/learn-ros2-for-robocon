{
  inputs = {
    flake-utils.url = "github:numtide/flake-utils";
    rosnix.url = "github:Pylgos/rosnix/release";
  };

  outputs =
    {
      self,
      flake-utils,
      rosnix,
    }:
    flake-utils.lib.eachDefaultSystem (
      system:
      let
        pkgs = rosnix.legacyPackages.${system}.jazzy;
        inherit (pkgs) rosPackages;
      in
      {
        devShells.default = rosPackages.mkRosWorkspaceShell {
          name = "learn-ros2-for-robocon-ws";
          buildInputs = [
            rosPackages.desktop
            rosPackages.ros-gz
            pkgs.rosPythonPackages.sympy
          ];
        };
      }
    );
}
