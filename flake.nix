{
  description = "Analytical inverse geometry for 6 links kinematic chains";

  inputs = {
    gepetto.url = "github:gepetto/nix";
    flake-parts.follows = "gepetto/flake-parts";
    systems.follows = "gepetto/systems";
  };

  outputs =
    inputs:
    inputs.flake-parts.lib.mkFlake { inherit inputs; } (
      { lib, ... }:
      {
        systems = import inputs.systems;
        imports = [
          inputs.gepetto.flakeModule
          {
            flakoboros = {
              extraDevPyPackages = [ "aig" ];
              overrideAttrs.aig = _: {
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
          }
        ];
      }
    );
}
