# MapRegionUpdate.msg

# One observation snapshot from a source.
MapObservationSource source

# Remove active observations from this source and map before applying patches.
# This is for source shutdowns, map transitions, or snapshots that replace the
# previous snapshot from the same source. It has no TTL because it is not an
# active map observation.
bool reset_source

# Patches from the same observation snapshot. The map server applies clear
# patches before obstacle patches so older same-source obstacles are removed
# along clear rays before current obstacle endpoints are marked.
MapRegionPatch[] patches
