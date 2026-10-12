declare module "*.svg" { const url: string; export default url; }
declare module "*.png" { const url: string; export default url; }
declare module "*.module.scss" { const classes: Record<string, string>; export default classes; }
