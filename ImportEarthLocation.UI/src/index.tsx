import { ModRegistrar } from "cs2/modding";
import { ImportEarthLocation } from "mods/import-earth-location";

const register: ModRegistrar = (moduleRegistry) => {
  console.info("ImportEarthLocation UI module registered in Editor slot");
  moduleRegistry.append("Editor", ImportEarthLocation);
};

export default register;
