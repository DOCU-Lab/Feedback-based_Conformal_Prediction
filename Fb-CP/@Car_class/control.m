function control(obj)
    %control function
    if obj.method_name == "Scp"
        control_Scp(obj);
    elseif  obj.method_name == "FbCpAra"
        control_FbCpAra(obj);
    elseif  obj.method_name == "FbCpIra"
        control_FbCpIra(obj);
    else
        error("Error method number")
    end
end