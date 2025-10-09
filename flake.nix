{
  description = "Analytical inverse geometry for 6 links kinematic chains";

  inputs = {
    gepetto.url = "github:MaximilienNaveau/nix/topic/mnaveau/add-aig";
    flake-parts.follows = "gepetto/flake-parts";
    nixpkgs.follows = "gepetto/nixpkgs";
    nix-ros-overlay.follows = "gepetto/nix-ros-overlay";
    systems.follows = "gepetto/systems";
    treefmt-nix.follows = "gepetto/treefmt-nix";
  };

  outputs =
    inputs:
    inputs.flake-parts.lib.mkFlake { inherit inputs; } {
      systems = import inputs.systems;
      imports = [ inputs.gepetto.flakeModule ];
      perSystem =
        {
          lib,
          pkgs,
          self',
          ...
        }:
        let
          my-src = lib.fileset.toSource {
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
        in
        {
          packages = {
            default = self'.packages.py-aig;
            aig = pkgs.aig.overrideAttrs {
              src = my-src;
            };
            py-aig = pkgs.python3Packages.aig.overrideAttrs {
              src = my-src;
            };
          };
        };
    };
}
