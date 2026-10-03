# Third-party data and references

## OpenStreetMap

Network data is downloaded from OpenStreetMap through public Overpass API instances.

Copyright OpenStreetMap contributors. Data is available under the Open Data Commons Open Database License (ODbL). A map made with this importer should include appropriate OpenStreetMap attribution when it is distributed or shown publicly.

https://www.openstreetmap.org/copyright

## Mapzen Terrarium elevation tiles

Elevation tiles are fetched from the public AWS terrain tile bucket using the Terrarium RGB encoding. The software does not bundle terrain data; it downloads and caches only the tiles needed for a user-selected location.

https://github.com/tilezen/joerd/blob/master/docs/formats.md

## CS2 community source references

The package uses the official mod-template shape tracked by `CitiesSkylinesModding/StockModTemplatesDiffer` and the documented native `CreationDefinition + NetCourse` workflow. Names of vanilla road and track prefabs are runtime-resolved; no game assets or assemblies are redistributed.

## Natural Earth populated places

`ImportEarthLocation/Data/cities.json` is a compact, bundled derivation of Natural Earth's 10m populated-places GeoJSON, acquired 2026-10-03. It contains name, country/region, coordinates, population, scale rank and capital flags. Selection retains population >=250,000, national capitals, or scale rank <=4; these are map-label metadata, not a current census or an airport dataset. Runtime selection is versioned independently as `metadata-v1`.

Source: https://github.com/nvkelso/natural-earth-vector/blob/master/geojson/ne_10m_populated_places.geojson

Natural Earth data is public domain: https://www.naturalearthdata.com/about/terms-of-use/

## Interactive map and place search

Map previews request only visible OpenStreetMap raster tiles, identify this application, retain a local cache, and show OpenStreetMap attribution. No viewport prefetch or import-area tile harvesting is performed.

Tile policy: https://operations.osmfoundation.org/policies/tiles/

Primary search: Photon (OpenStreetMap): https://github.com/komoot/photon

Fallback search: Open-Meteo geocoding (GeoNames): https://open-meteo.com/en/docs/geocoding-api

The picker shows provider attribution. Public demo/free endpoint availability and permitted commercial use must be reviewed before distributing a commercial product.

## Climate normals

NASA POWER no-key climatology is requested for the selected location (fixed 2001–2020 climatology, temperature/precipitation/10m wind speed/direction), validated and cached locally. This is not a bundled global offline climate dataset. Source/API version is included in the import report. Meteorological wind direction means FROM; the native flow vector is converted accordingly.

https://power.larc.nasa.gov/docs/services/api/temporal/climatology/

https://power.larc.nasa.gov/docs/methodology/meteorology/wind/
