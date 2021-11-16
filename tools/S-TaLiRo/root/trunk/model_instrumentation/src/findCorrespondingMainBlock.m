function [ mainBlockId ] = findCorrespondingMainBlock(blockId, info)
%findCorrespondingMainBlock Searches inside info.mainBlockIndices for the
%given blockId.

try
    mainBlockId = -1;
    for i = 1:info.numOfMainBlocks
        if ~isempty(find(info.mainBlockIndices{i} == blockId, 1))
            mainBlockId = i;
        end
    end
catch
    error('ERROR: findCorrespondingMainBlock failed!');
end

end

