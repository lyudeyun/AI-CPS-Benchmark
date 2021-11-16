% Convert hydis object to string (e.g. for printing). 
% Not equivalent to display overloaded method. 
function str = num2str(obj)

out = get(obj);
str = ['<',num2str(out(1)),',',num2str(out(2)),'>'];
