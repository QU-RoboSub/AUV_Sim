function msg = std_msgs_MultiArrayDimensionStruct
% Message struct definition for std_msgs/MultiArrayDimension
coder.inline("never")
msg = struct(...
    'MessageType','std_msgs/MultiArrayDimension',...
    'Label',ros.internal.ros.messages.ros.char('string',0),...
    'Size',ros.internal.ros.messages.ros.default_type('uint32',1),...
    'Stride',ros.internal.ros.messages.ros.default_type('uint32',1));
coder.cstructname(msg,'std_msgs_MultiArrayDimensionStruct_T');
coder.varsize('msg.Label',[1 1000000000],[0 1]);
if ~isempty(coder.target)
    coder.ceval('//',coder.rref(msg));
end
end
