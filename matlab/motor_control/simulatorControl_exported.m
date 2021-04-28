classdef simulatorControl_exported < matlab.apps.AppBase

    % Properties that correspond to app components
    properties (Access = public)
        UIFigure     matlab.ui.Figure
        UpButton     matlab.ui.control.Button
        DownButton   matlab.ui.control.Button
        SliderLabel  matlab.ui.control.Label
        Slider       matlab.ui.control.Slider
    end

    % Callbacks that handle component events
    methods (Access = private)

        % Button pushed function: UpButton
        function UpButtonPushed(app, event)
            app.Slider.Value = app.Slider.Value +1;
        end
    end

    % Component initialization
    methods (Access = private)

        % Create UIFigure and components
        function createComponents(app)

            % Create UIFigure and hide until all components are created
            app.UIFigure = uifigure('Visible', 'off');
            app.UIFigure.Position = [100 100 640 480];
            app.UIFigure.Name = 'MATLAB App';

            % Create UpButton
            app.UpButton = uibutton(app.UIFigure, 'push');
            app.UpButton.ButtonPushedFcn = createCallbackFcn(app, @UpButtonPushed, true);
            app.UpButton.Position = [58 412 100 22];
            app.UpButton.Text = 'Up';

            % Create DownButton
            app.DownButton = uibutton(app.UIFigure, 'push');
            app.DownButton.Position = [58 360 100 22];
            app.DownButton.Text = 'Down';

            % Create SliderLabel
            app.SliderLabel = uilabel(app.UIFigure);
            app.SliderLabel.HorizontalAlignment = 'right';
            app.SliderLabel.Position = [273 402 36 22];
            app.SliderLabel.Text = 'Slider';

            % Create Slider
            app.Slider = uislider(app.UIFigure);
            app.Slider.Position = [330 411 150 3];

            % Show the figure after all components are created
            app.UIFigure.Visible = 'on';
        end
    end

    % App creation and deletion
    methods (Access = public)

        % Construct app
        function app = simulatorControl_exported

            % Create UIFigure and components
            createComponents(app)

            % Register the app with App Designer
            registerApp(app, app.UIFigure)

            if nargout == 0
                clear app
            end
        end

        % Code that executes before app deletion
        function delete(app)

            % Delete UIFigure when app is deleted
            delete(app.UIFigure)
        end
    end
end