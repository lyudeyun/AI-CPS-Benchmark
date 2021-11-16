function [ parentIndices ] = getParentIndices( parentHandles, handles )
%getParentIndices Convert parent handles to indices.

try
    parentIndices = zeros(1, numel(parentHandles));
    for i = 1:numel(parentHandles)
        parentIndices(i) = getIndexFromHandle(parentHandles(i), handles);
    end
catch
    error('ERROR: getParentIndices failed!');
end
end

