function msg = std_msgs_MultiArrayLayoutStruct
% Message struct definition for std_msgs/MultiArrayLayout
coder.inline("never")
msg = struct(...
    'MessageType','std_msgs/MultiArrayLayout',...
    'Dim',std_msgs_MultiArrayDimensionStruct,...
    'DataOffset',ros.internal.ros.messages.ros.default_type('uint32',1));
coder.cstructname(msg,'std_msgs_MultiArrayLayoutStruct_T');
coder.varsize('msg.Dim',[1000000000 1],[1 0]);
msg.Dim = msg.Dim([],1);
if ~isempty(coder.target)
    coder.ceval('//',coder.rref(msg));
end
end
