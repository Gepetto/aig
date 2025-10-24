{
  description = "Analytical inverse geometry for 6 links kinematic chains";

  inputs = {
    gepetto.url = "github:gepetto/nix";
    flake-parts.follows = "gepetto/flake-parts";
    nixpkgs.follows = "gepetto/nixpkgs";
    nix-ros-overlay.follows = "gepetto/nix-ros-overlay";
    systems.follows = "gepetto/systems";
    treefmt-nix.follows = "gepetto/treefmt-nix";
  };

  outputs =
    inputs:
    inputs.flake-parts.lib.mkFlake { inherit inputs; } (
      { lib, self, ... }:
      {
        systems = import inputs.systems;
        imports = [
          inputs.gepetto.flakeModule
          { gepetto-pkgs.overlays = [ self.overlays.default ]; }
        ];
        flake.overlays.default = _final: prev: {
          aig = prev.aig.overrideAttrs {
            src = lib.fileset.toSource {
              root = ./.;
              fileset = lib.fileset.unions [
                ./CMakeLists.txt
                ./include
                ./package.xml
                ./python
                ./src
                ./tests
              ];
            };
          };
        };
        perSystem =
          { pkgs, self', ... }:
          {
            packages = {
              default = self'.packages.aig;
              aig = pkgs.python3Packages.aig.override { standalone = false; };
            };
          };
      }
    );
}
