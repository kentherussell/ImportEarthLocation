import { ModRegistrar } from "cs2/modding";
import { ImportEarthLocation } from "mods/import-earth-location";

const register: ModRegistrar = (moduleRegistry) => {
  console.info("ImportEarthLocation UI module registered in Editor slot");
  moduleRegistry.append("Editor", ImportEarthLocation);
  // Never depend on another mod's Toolbar React element/wrapper shape.
  // The editor overlay owns one launcher and anchors it to the rendered row.
};

export default register;
