function isLeaf = isBlockLeaf(block, connMatrix)

if isempty(find(connMatrix(block, :), 1))
    isLeaf = 1;
else
    isLeaf = 0;
end
end
