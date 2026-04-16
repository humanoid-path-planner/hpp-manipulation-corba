{
  description = "Corba server for manipulation planning";

  inputs.gepetto.url = "github:gepetto/nix";

  outputs =
    inputs:
    inputs.gepetto.lib.mkFlakoboros inputs (
      { lib, ... }:
      {
        overrideAttrs.hpp-manipulation-corba = {
          src = lib.fileset.toSource {
            root = ./.;
            fileset = lib.fileset.unions [
              ./CMakeLists.txt
              ./doc
              ./idl
              ./include
              ./package.xml
              ./src
              ./tests
            ];
          };
        };
      }
    );
}
