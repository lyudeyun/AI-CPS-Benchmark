function num = numOfBlocks(info)

if isfield(info, 'numOfBlocks')
    num = info.numOfBlocks;
else
    num = length(info.connMatrix);
end

end

