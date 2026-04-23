import { BackendProvider } from './context/BackendContext';
import Dashboard from './components/Dashboard';

function App() {
  return (
    <BackendProvider>
      <Dashboard />
    </BackendProvider>
  );
}

export default App;
