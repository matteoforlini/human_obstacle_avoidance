function ImpostazioneView

    grid on
    axis equal
    xlabel('X');
    ylabel('Y');
    zlabel('Z');
    view([45 30]);
    xlim([-2 2])
    ylim([-2 2])
    zlim([0 2])
%     camroll(180) % Per il robot rovesciato
end
