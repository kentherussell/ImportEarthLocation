import { ModRegistrar } from "cs2/modding";
import { ImportEarthLocation } from "mods/import-earth-location";

const register: ModRegistrar = (moduleRegistry) => {
  moduleRegistry.append("Editor", ImportEarthLocation);
};

export default register;
