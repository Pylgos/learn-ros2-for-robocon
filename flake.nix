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
        mkRosWorkspaceShell = rosPackages.mkRosWorkspaceShell.override {
          buildColconPackage = rosPackages.buildColconPackage.override {
            stdenv = pkgs.clangStdenv;
          };
        };
      in
      {
        devShells.default = mkRosWorkspaceShell {
          name = "learn-ros2-for-robocon-ws";
          nativeBuildInputs = [
            pkgs.cargo
          ];
          buildInputs = [
            rosPackages.desktop
            rosPackages.pcl-ros
            rosPackages.ros-gz
            pkgs.rosPythonPackages.sympy
          ];
          LIBCLANG_PATH = "${pkgs.llvmPackages.clang-unwrapped.lib}/lib";
        };
      }
    );
}
