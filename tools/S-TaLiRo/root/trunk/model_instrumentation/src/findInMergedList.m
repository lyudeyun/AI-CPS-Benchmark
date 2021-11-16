function [ mergedListId ] = findInMergedList(blockId, mergedList)
%findInMergedList Searches inside mergedList for the given blockId.

try
    mergedListId = -1;
    for i = 1:numel(mergedList)
        if ~isempty(find(mergedList{i} == blockId, 1))
            mergedListId = i;
        end
    end
catch
    error('ERROR: findInMergedList failed!');
end

end

