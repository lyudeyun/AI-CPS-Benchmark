function [ names ] = readMainBlockNames( info )
%UNTITLED Summary of this function goes here
%   Detailed explanation goes here

names = cell(info.numOfMainBlocks,1);
for mainBlockId = 1:info.numOfMainBlocks
    blockName = [];
    nameSeparator = [];
    for m = info.mainBlockIndices{mainBlockId} %m gets the original block index
        blockName = [blockName, nameSeparator];
        tempName = info.blockList{m};
        splitName = strsplit(tempName, '/');
        blockName = [blockName, splitName{end}];
        nameSeparator = ',';
    end
    names{mainBlockId} = blockName;
end

