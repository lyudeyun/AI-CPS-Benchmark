function i = getIndexB(block, core, info)

i = info.startOfB + ((block - 1) * info.numOfCores) + core - 1;
end

